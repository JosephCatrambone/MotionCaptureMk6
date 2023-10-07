import boofcv.factory.fiducial.ConfigFiducialHammingDetector
import boofcv.factory.fiducial.ConfigHammingMarker
import boofcv.factory.fiducial.FactoryFiducial
import boofcv.factory.fiducial.HammingDictionary
import boofcv.kotlin.asGrayU8
import boofcv.kotlin.asNarrowDistortion
import boofcv.struct.calib.CameraPinholeBrown
import boofcv.struct.image.GrayU8
import java.awt.image.BufferedImage
import kotlin.math.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*
import java.util.concurrent.atomic.AtomicBoolean

suspend fun detectFiducials(quit: AtomicBoolean, imageStream: Channel<BufferedImage>, intrinsics: CameraPinholeBrown, frameBuffer:CircularReferenceBuffer<AnnotatedImage>) {
	val detector = FactoryFiducial.squareHamming(ConfigHammingMarker.loadDictionary(HammingDictionary.APRILTAG_36h10), ConfigFiducialHammingDetector(), GrayU8::class.java)
	detector.setLensDistortion(intrinsics.asNarrowDistortion(), intrinsics.width, intrinsics.height)
	assert(detector.is3D)

	var frameCount = 0

	while(!quit.get() && !imageStream.isClosedForReceive) {
		var ref: AnnotatedImage? = null;
		while(ref == null) {
			yield()
			ref = frameBuffer.writeNext()
		}

		val image = imageStream.receive()

		frameCount++

		// For QR
		//detector.process(image.asGrayU8())
		detector.detect(image.asGrayU8())
		ref.frameNumber = frameCount
		ref.detections = detector.totalFound()
		ref.img = image
		for(i in 0..< min(MAX_DETECTIONS, ref.detections)) {
			ref.ids[i] = detector.getId(i)
			detector.getCenter(i, ref.pixel[i])
			detector.getBounds(i, ref.bounds[i])
			detector.getFiducialToCamera(i, ref.pose[i])
		}
		//frameBuffer.readNext() // Mark as read?
	}
}