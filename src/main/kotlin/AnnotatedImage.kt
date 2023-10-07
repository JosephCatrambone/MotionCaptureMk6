import georegression.struct.point.Point2D_F64
import georegression.struct.se.Se3_F64
import georegression.struct.shapes.Polygon2D_F64
import java.awt.image.BufferedImage

class AnnotatedImage(
	var img: BufferedImage,
	var detections: Int,
	var frameNumber: Int,
	var frameTime: Float,
	val ids: MutableList<Long>,
	val pose: List<Se3_F64>,
	// These are both extra annotations
	val bounds: List<Polygon2D_F64>,
	val pixel: List<Point2D_F64>,
) {
	companion object {
		fun preallocate(img: BufferedImage): AnnotatedImage {
			return AnnotatedImage(
				img = img,
				detections = 0,
				frameNumber = 0,
				frameTime = 0.0F,
				MutableList(MAX_DETECTIONS) { 0 },
				MutableList(MAX_DETECTIONS) { Se3_F64() },
				MutableList(MAX_DETECTIONS) { Polygon2D_F64() },
				MutableList(MAX_DETECTIONS) { Point2D_F64() }
			)
		}
	}
}
