import boofcv.abst.fiducial.calib.ConfigGridDimen
import boofcv.abst.geo.calibration.CalibrateMonoPlanar
import boofcv.factory.fiducial.FactoryFiducialCalibration
import boofcv.gui.feature.VisualizeFeatures
import boofcv.gui.image.ImagePanel
import boofcv.gui.image.ShowImages
import boofcv.io.calibration.CalibrationIO
import boofcv.io.webcamcapture.UtilWebcamCapture
import boofcv.kotlin.asGrayF32
import boofcv.struct.calib.CameraPinholeBrown
import georegression.struct.point.Point2D_F64
import java.awt.Color
import java.awt.image.BufferedImage
import kotlin.math.*


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