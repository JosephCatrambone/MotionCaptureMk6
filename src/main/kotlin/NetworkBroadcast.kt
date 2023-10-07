import boofcv.kotlin.asQuaternion
import com.sun.net.httpserver.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.encodeToString
import java.io.OutputStream
import java.net.*
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean


/***
 * WireSingleFiducialDetection doesn't have the frame number or frame time because it's part of a larger object.
 */
@Serializable
data class WireSingleFiducialDetection(
	val id: Int,
	val position: List<Float>,
	val rotation: List<Float>,
)

@Serializable
data class WireFrame(
	val frameNumber: Int,
	val frameTime: Float,
	val detections: List<WireSingleFiducialDetection>,
)



@OptIn(ExperimentalCoroutinesApi::class)
fun CoroutineScope.broadcastFiducialPoses(quit: AtomicBoolean, bindPort: Int, annotationBuffer: CircularReferenceBuffer<AnnotatedImage>): ReceiveChannel<AnnotatedImage> = produce {
	// TODO: Send out the image and then maybe push it onto the annotated image pile.
	//val httpServer = HttpServer.create(InetSocketAddress(bindPort), 0)
	//httpServer.createContext("/", PoseBroadcastHandler)

	while(!quit.get()) {
		val data = annotationBuffer.readNext()
		if(data != null) {
			send(data)
		} else {
			delay(1)
		}
	}
}


class PoseBroadcastHandler(val networkPackets: WireFrame): HttpHandler {
	override fun handle(exchange: HttpExchange?) {
		val response = Json.encodeToString(networkPackets)
		exchange!!.sendResponseHeaders(200, response.length.toLong())
		val os: OutputStream = exchange.responseBody
		os.write(response.toByteArray())
		os.close()
	}
}


suspend fun startSendUDP(port: Int, annotationBuffer: CircularReferenceBuffer<AnnotatedImage>) {
	// Reminder: maximum UDP packet size: 64kb. (65507 after overhead.  65535 before.)
	// 64kb / 512 markers max = 128 bytes per marker max.
	// Need 7 floats + 1 int (id + 3dof pos, 4dof quaternion).  Could just do euler for rotation?
	// 8 floats * 8bytes each = 64bytes/marker.  256 markers max + overhead for time and frame.

	//val server = DatagramSocket(bindPort)
	val server = MulticastSocket(port)
	//server.networkInterface = NetworkInterface.getByName("192.168.2.152")
	var interfaceIdx = 0
	while(NetworkInterface.getByIndex(interfaceIdx) == null) {
		interfaceIdx++
		if(interfaceIdx >= 255) {
			throw Exception("Couldn't find network interface.  Tried 255.")
		}
	}
	server.networkInterface = NetworkInterface.getByIndex(interfaceIdx)
	println("Using network interface ${server.networkInterface.displayName}")
	//server.joinGroup(InetAddress.getByName("224.0.0.69")) // http://www.iana.org/assignments/multicast-addresses/multicast-addresses.xhtml
	val outputBufferSize = (3*8) + (8*8)*MAX_DETECTIONS
	val outputBuffer = ByteArray(outputBufferSize)

	val data = annotationBuffer.readNext()
	if(data != null) {
		val conversionBuffer = ByteBuffer.wrap(outputBuffer) // Frame ID, time float, detection count
		conversionBuffer.putInt(data.frameNumber)
		conversionBuffer.putFloat(data.frameTime)
		conversionBuffer.putInt(data.detections)
		for (i in 0..data.detections) {
			val tx = data.pose[i].translation
			val rx = data.pose[i].rotation.asQuaternion()

			conversionBuffer.putFloat(tx.x.toFloat())
			conversionBuffer.putFloat(tx.y.toFloat())
			conversionBuffer.putFloat(tx.z.toFloat())

			conversionBuffer.putFloat(rx.x.toFloat())
			conversionBuffer.putFloat(rx.y.toFloat())
			conversionBuffer.putFloat(rx.z.toFloat())
			conversionBuffer.putFloat(rx.w.toFloat())
		}
		// Can drop 'withContext' if we merge this again.
		withContext(Dispatchers.IO) {
			server.send(DatagramPacket(outputBuffer, outputBufferSize))
		}
	}
}
