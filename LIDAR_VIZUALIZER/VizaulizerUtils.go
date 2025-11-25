package lidarvizualizer
import (
	"gocv.io/x/gocv"
)

func Color(r, g, b, a int) [4]float64 {
    return [4]float64{
        float64(b),
        float64(g),
        float64(r),
        float64(a),
    }
}

func FormatSolidImage( width int, height int, colorData [4] float64)  (img gocv.Mat){
	img = gocv.NewMatWithSize(height, width, gocv.MatTypeCV8UC3)
	color := gocv.NewScalar(colorData[0], colorData[1], colorData[2], colorData[3])
	img.SetTo(color)
	return
}
