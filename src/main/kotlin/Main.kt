import boofcv.abst.fiducial.calib.ConfigGridDimen
import boofcv.abst.geo.calibration.CalibrateMonoPlanar
import boofcv.factory.fiducial.ConfigFiducialHammingDetector
import boofcv.factory.fiducial.ConfigHammingMarker
import boofcv.factory.fiducial.FactoryFiducial
import boofcv.factory.fiducial.FactoryFiducialCalibration
import boofcv.factory.fiducial.HammingDictionary
import boofcv.gui.feature.VisualizeFeatures
import boofcv.gui.fiducial.VisualizeFiducial
import boofcv.gui.image.ImagePanel
import boofcv.gui.image.ShowImages
import boofcv.io.calibration.CalibrationIO
import boofcv.io.webcamcapture.UtilWebcamCapture
import boofcv.kotlin.asGrayF32
import boofcv.kotlin.asGrayU8
import boofcv.kotlin.asNarrowDistortion

import boofcv.struct.calib.CameraPinholeBrown
import boofcv.struct.image.GrayU8
import georegression.struct.point.Point2D_F64
import georegression.struct.se.Se3_F64
import georegression.struct.shapes.Polygon2D_F64
import java.awt.BasicStroke
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.UncheckedIOException
import kotlin.math.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*
import java.util.concurrent.atomic.AtomicBoolean

const val DEFAULT_BIND_PORT = 42069
const val CALIBRATION_FILENAME = "intrinsics.yaml"
const val FIDUCIAL_BUFFER_SIZE = 120
const val MAX_DETECTIONS = 254  // Capped because we need to fit all of it in a UDP packet.
const val ANNOTATION_WORKERS = 1
const val DISPLAY_BUFFER_SIZE = 30




fun main(args: Array<String>) = runBlocking {
	//val tagDirectory: String = UtilIO.pathExample("fiducial/square_hamming/aruco_25h7")
	val width = 640
	val height = 360
	val quit = AtomicBoolean(false)

	// Try loading calibration OR calibrate
	val intrinsics = try {
		CalibrationIO.load<CameraPinholeBrown>(CALIBRATION_FILENAME)
	} catch(e: UncheckedIOException) {
		runCalibration(width, height)
	}

	//val intrinsics = CameraPinholeBrown(640.0, 360.0, 0.0, 640.0, 360.0, 1280, 720)
	//val distortion: LensDistortionNarrowFOV = LensDistortionBrown(intrinsics)

	// frameGenerator -> { many annotatedImageStream generators } -> | broadcast to UDP and annotate in parallel |
	// (frameGenerator -> fiducial annotation -> ring buffer) and in parallel (ring buffer -> broadcast -> render)
	val webcamStream = Channel<BufferedImage>()
	val frameBuffer = CircularReferenceBuffer<AnnotatedImage>(FIDUCIAL_BUFFER_SIZE) { AnnotatedImage.preallocate(BufferedImage(1, 1, BufferedImage.TYPE_INT_RGB)) }
	val annotatedImageStream = Channel<BufferedImage>(DISPLAY_BUFFER_SIZE, onBufferOverflow = BufferOverflow.DROP_OLDEST)

	launch {
		generateImageStream(quit, intrinsics, webcamStream)
	}
	repeat(ANNOTATION_WORKERS) {
		launch {
			detectFiducials(quit, webcamStream, intrinsics, frameBuffer) // Pushes to the frame buffer
		}
	}
	launch {
		val poseData = broadcastFiducialPoses(quit, DEFAULT_BIND_PORT, frameBuffer)
		annotateImages(intrinsics, poseData, annotatedImageStream)
	}

	val gui = ImagePanel()
	gui.preferredSize = java.awt.Dimension(width, height) //webcam.viewSize
	ShowImages.showWindow(gui, "TITLE!", true)
	for(img in annotatedImageStream) {
		//val annotatedImage: AnnotatedImage = localizations.receive()
		gui.setImageRepaint(img)
	}
}

fun withFrameIterator(width: Int, height: Int, f: (img: BufferedImage) -> Unit) {
	val webcam = UtilWebcamCapture.openDefault(width, height)

	while (true) {
		val image = webcam.image ?: break
		f(image)
	}
}

suspend fun generateImageStream(quit: AtomicBoolean, intrinsics: CameraPinholeBrown, channel: Channel<BufferedImage>) {
	val webcam = UtilWebcamCapture.openDefault(intrinsics.width, intrinsics.height)
	while(!quit.get() && !channel.isClosedForSend) {
		val image = webcam.image ?: break
		channel.send(image)
	}
}




suspend fun annotateImages(intrinsics: CameraPinholeBrown, annotatedImageStream: ReceiveChannel<AnnotatedImage>, outputStream: Channel<BufferedImage>) {
	for(annotatedImage in annotatedImageStream) {
		val image: BufferedImage = annotatedImage.img
		val g2 = image.createGraphics()
		g2.color = Color.RED
		g2.stroke = BasicStroke(4.0f)
		//for (qr in detector.detections) {
		for (i in 0..<annotatedImage.detections) {
			val ident = annotatedImage.ids[i]
			//VisualizeShapes.drawPolygon(bounds, true, 1.0, g2)
			VisualizeFiducial.drawCube(annotatedImage.pose[i], intrinsics, 1.0, 1, g2)
			VisualizeFiducial.drawLabelCenter(annotatedImage.pose[i], intrinsics, ". ID:${ident}", g2)
		}
		outputStream.send(image)
	}

}

