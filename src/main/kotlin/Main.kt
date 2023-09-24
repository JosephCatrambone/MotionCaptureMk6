import boofcv.abst.fiducial.calib.ConfigECoCheckDetector
import boofcv.abst.fiducial.calib.ConfigECoCheckMarkers
import boofcv.abst.fiducial.calib.ConfigGridDimen
import boofcv.abst.geo.calibration.CalibrateMonoPlanar
import boofcv.abst.geo.calibration.MultiToSingleFiducialCalibration
import boofcv.alg.distort.LensDistortionNarrowFOV
import boofcv.alg.distort.brown.LensDistortionBrown
import boofcv.factory.fiducial.ConfigFiducialHammingDetector
import boofcv.factory.fiducial.ConfigHammingMarker
import boofcv.factory.fiducial.FactoryFiducial
import boofcv.factory.fiducial.FactoryFiducialCalibration
import boofcv.factory.fiducial.HammingDictionary
import boofcv.gui.feature.VisualizeFeatures
import boofcv.gui.feature.VisualizeShapes
import boofcv.gui.fiducial.VisualizeFiducial
import boofcv.gui.image.ImagePanel
import boofcv.gui.image.ShowImages
import boofcv.io.UtilIO
import boofcv.io.calibration.CalibrationIO
import boofcv.io.webcamcapture.UtilWebcamCapture
import boofcv.kotlin.asGrayF32
import boofcv.kotlin.asGrayU8
import boofcv.kotlin.asNarrowDistortion
import boofcv.struct.calib.CameraPinholeBrown
import boofcv.struct.image.GrayU8
import com.github.sarxos.webcam.Webcam
import com.github.sarxos.webcam.WebcamUtils
import georegression.struct.point.Point2D_F64
import georegression.struct.se.Se3_F64
import georegression.struct.shapes.Polygon2D_F64
import java.awt.BasicStroke
import java.awt.Color
import java.awt.image.BufferedImage
import java.io.UncheckedIOException
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*

const val CALIBRATION_FILENAME = "intrinsics.yaml"


fun main(args: Array<String>) = runBlocking {
	//val tagDirectory: String = UtilIO.pathExample("fiducial/square_hamming/aruco_25h7")
	val width = 1280
	val height = 720
	//val imageChannel = Channel<BufferedImage>()

	// Try loading calibration OR calibrate
	val intrinsics = try {
		CalibrationIO.load<CameraPinholeBrown>(CALIBRATION_FILENAME)
	} catch(e: UncheckedIOException) {
		runCalibration(1280, 720)
	}

	//val intrinsics = CameraPinholeBrown(640.0, 360.0, 0.0, 640.0, 360.0, 1280, 720)
	//val distortion: LensDistortionNarrowFOV = LensDistortionBrown(intrinsics)

	val quit: AtomicBoolean = AtomicBoolean(false)
	//val imageStream = produceImageStream(quit, 1280, 720)
	val webcam = UtilWebcamCapture.openDefault(width, height)

	val gui = ImagePanel()
	gui.preferredSize = java.awt.Dimension(1280, 720) //webcam.viewSize
	ShowImages.showWindow(gui, "TITLE!", true)

	// Reminder: maximum UDP packet size: 16kb.
	//val param: CameraPinholeBrown = CalibrationIO.load();
	//LensDistortionNarrowFOV lensDistortion = new LensDistortionBrown(param);

	//val detector = FactoryFiducial.qrcode(null, GrayU8::class.java)
	val detector = FactoryFiducial.squareHamming(ConfigHammingMarker.loadDictionary(HammingDictionary.APRILTAG_36h10), ConfigFiducialHammingDetector(), GrayU8::class.java)
	detector.setLensDistortion(intrinsics.asNarrowDistortion(), webcam.viewSize.width, webcam.viewSize.height)
	assert(detector.is3D)

	val bounds = Polygon2D_F64();
	val targetToSensor = Se3_F64();
	val locationPixel = Point2D_F64();
	while (true) {
		val image = webcam.image ?: break

		// For QR
		//detector.process(image.asGrayU8())
		detector.detect(image.asGrayU8())

		val g2 = image.createGraphics()
		g2.color = Color.RED
		g2.stroke = BasicStroke(4.0f)
		//for (qr in detector.detections) {
		for(i in 0 ..< detector.totalFound()) {
			val ident = detector.getId(i)
			detector.getCenter(i, locationPixel)
			detector.getBounds(i, bounds)
			// is3D:
			detector.getFiducialToCamera(i, targetToSensor)
			//VisualizeShapes.drawPolygon(bounds, true, 1.0, g2)
			VisualizeFiducial.drawCube(targetToSensor, intrinsics, 1.0, 1, g2)
			VisualizeFiducial.drawLabelCenter(targetToSensor, intrinsics, ". ID:${ident}", g2)
		}

		gui.setImageRepaint(image)
	}
}

fun withFrameIterator(width: Int, height: Int, f: (img: BufferedImage) -> Unit) {
	val webcam = UtilWebcamCapture.openDefault(width, height)

	while (true) {
		val image = webcam.image ?: break
		f(image)
	}
}

@OptIn(ExperimentalCoroutinesApi::class)
fun CoroutineScope.produceImageStream(quit: AtomicBoolean, width: Int, height: Int): ReceiveChannel<BufferedImage> = produce {
	val webcam = UtilWebcamCapture.openDefault(width, height)
	while(!quit.get()) {
		val image = webcam.image ?: break
		send(image)
	}
}

fun runCalibration(width: Int, height: Int): CameraPinholeBrown {
	val webcam = UtilWebcamCapture.openDefault(width, height)

	val gui = ImagePanel()
	gui.preferredSize = webcam.viewSize
	ShowImages.showWindow(gui, "Calibration", true)

	val spacing = 20.0
	val yIntersections = 11 // One less than the number of 'columns'.
	val xIntersections = 9
	val worldPoints = mutableListOf<Point2D_F64>() // Boof assumes right-handed z-forward, which means +y is down, right?
	for(y in 0..yIntersections+1) {
		for(x in 0..xIntersections+1) {
			worldPoints.add(Point2D_F64(x * spacing, y * spacing))
		}
	}
	//val detector = MultiToSingleFiducialCalibration(FactoryFiducial.ecocheck(null, ConfigECoCheckMarkers.singleShape(9, 7, 1, 30)))
	val detector = FactoryFiducialCalibration.chessboardX(null, ConfigGridDimen(yIntersections+1, xIntersections+1, spacing)) // 11,9 -> Finding 8x10 intersections
	val calibrator = CalibrateMonoPlanar(worldPoints)
	calibrator.configurePinhole(true, 2, false)
	calibrator.reset()

	val selectedImages = mutableListOf<BufferedImage>()
	var frameSkips: Int = 0
	while(true) {
		val image = webcam.image ?: break;
		val g2 = image.createGraphics()
		g2.drawString("Camera Uncalibrated - Capturing ${xIntersections}x${yIntersections} chess board", 100, 100)
		if(frameSkips == 0 && detector.process(image.asGrayF32())) {
			if(selectedImages.isEmpty()) {
				// TODO: Init calibrator?
			}
			val points = detector.detectedPoints.copy()
			calibrator.addImage(points)
			selectedImages.add(image)
			points.points.forEach {
				VisualizeFeatures.drawPoint(g2, it.p.x.toInt(), it.p.y.toInt(), 2, Color.RED)
			}
			frameSkips += 10 // Drop a few frames so we can reposition.
		} else {
			frameSkips = max(frameSkips-1, 0)
		}
		gui.setImageRepaint(image)
		if(selectedImages.count() > 50) {
			break;
		}
	}
	val intrinsics = calibrator.process<CameraPinholeBrown>()
	CalibrationIO.save(intrinsics, CALIBRATION_FILENAME)
	intrinsics.print()
	webcam.close()
	return intrinsics
}

/*
# Pinhole camera model with radial and tangential distortion
# (fx,fy) = focal length, (cx,cy) = principle point, (width,height) = image shape
# radial = radial distortion, (t1,t2) = tangential distortion

pinhole:
  fx: 632.6331008160633
  fy: 629.4057227776121
  cx: 401.16829030891256
  cy: 229.2684635210703
  width: 800
  height: 450
  skew: 0.0
model: pinhole_radial_tangential
radial_tangential:
  radial:
  - 0.07946796807805329
  - -0.1703344505939042
  t1: 0.0
  t2: 0.0
version: 0
 */