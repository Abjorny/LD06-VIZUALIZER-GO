package lidarvizualizer

import (
	"LIDAR/LIDAR_DRIVER"
	"gocv.io/x/gocv"
	"image"
	"image/color"
	"math"
	"sync"
)

type VizulizerLD06 struct {
	Points   *[360]lidardriver.ResultPoint
	Mu       *sync.Mutex
	Image    gocv.Mat
	Width    int
	Height   int
	MaxRange float64
}

func NewVizulizerLD06(
	width int,
	height int,
	mu *sync.Mutex,
	points *[360]lidardriver.ResultPoint,
	maxRange float64) *VizulizerLD06 {

	img := FormatSolidImage(width, height, Color(255, 255, 255, 0))
	center := image.Pt(width/2, height/2)

	radius := 10
	circleColor := color.RGBA{R: 255, G: 0, B: 0, A: 0}
	gocv.Circle(&img, center, radius, circleColor, 3)
	gocv.Circle(&img, center, width/2, circleColor, 3)
	font := gocv.FontHersheySimplex
	fontScale := 0.7
	thickness := 2

	gocv.Line(&img, image.Pt(0, height/2), image.Pt(width, height/2), circleColor, 3)
	gocv.Line(&img, image.Pt(width/2, 0), image.Pt(width/2, height), circleColor, 3)

	gocv.PutText(&img, "90", image.Pt(width/2+10, 30), font, fontScale, circleColor, thickness)
	gocv.PutText(&img, "0", image.Pt(10, height/2-10), font, fontScale, circleColor, thickness)
	gocv.PutText(&img, "180", image.Pt(width-50, height/2+30), font, fontScale, circleColor, thickness)
	gocv.PutText(&img, "270", image.Pt(width/2-50, height-10), font, fontScale, circleColor, thickness)

	return &VizulizerLD06{
		Points:   points,
		Mu:       mu,
		Image:    img,
		Width:    width,
		Height:   height,
		MaxRange: maxRange,
	}
}
func (v *VizulizerLD06) getSnapshot() [360]lidardriver.ResultPoint {
	v.Mu.Lock()
	defer v.Mu.Unlock()
	return *v.Points
}
func (v *VizulizerLD06) FormatMap(img gocv.Mat, pointsData []image.Point, scale float64) (outputImage gocv.Mat) {
	outputImage = FormatSolidImage(v.Width, v.Height, Color(0, 0, 0, 255)) // полностью черный

	pv := gocv.NewPointsVector()
	defer pv.Close()

	contour := gocv.NewPointVector()
	defer contour.Close()

	cx := v.Width / 2
	cy := v.Height / 2

	for _, p := range pointsData {
		dx := float64(p.X - cx)
		dy := float64(p.Y - cy)

		dx *= scale
		dy *= scale

		scaled := image.Point{
			X: int(dx) + cx,
			Y: int(dy) + cy,
		}

		contour.Append(scaled)
	}

	if contour.Size() > 0 {
		pv.Append(contour)
	}

	col := color.RGBA{R: 255, G: 255, B: 255, A: 255} 

	gocv.Circle(&outputImage, image.Pt(v.Width/2, v.Height/2), 2, col, 3)

	if pv.Size() > 0 {
		gocv.FillPoly(&outputImage, pv, col)
	}

	hsv := gocv.NewMat()
	defer hsv.Close()
	gocv.CvtColor(outputImage, &hsv, gocv.ColorBGRToHSV)

	lower := gocv.NewScalar(0, 0, 150, 0)
	upper := gocv.NewScalar(255, 255, 255, 0)
	lowerMat := gocv.NewMatFromScalar(lower, gocv.MatTypeCV8UC3)
	upperMat := gocv.NewMatFromScalar(upper, gocv.MatTypeCV8UC3)
	mask := gocv.NewMat()
	defer mask.Close()
	gocv.InRange(hsv, lowerMat, upperMat, &mask)

	contours := gocv.FindContours(mask, gocv.RetrievalExternal, gocv.ChainApproxSimple)
	defer contours.Close()

	var maxArea float64
	var maxContour gocv.PointVector
	found := false

	if contours.Size() > 0 {
		for i := 0; i < contours.Size(); i++ {
			c := contours.At(i)
			area := gocv.ContourArea(c)

			if area > maxArea {
				maxArea = area
				maxContour = c
				found = true
			}
		}
	}

	if found && maxContour.Size() > 0 {
		rect := gocv.BoundingRect(maxContour)
		gocv.Rectangle(&outputImage, rect, color.RGBA{255, 0, 0, 255}, 2)
	}

	return outputImage
}
func (v *VizulizerLD06) GetVizuliz() (img gocv.Mat, pointsData []image.Point) {
	img = v.Image.Clone()
	points := v.getSnapshot()
	cx := v.Width / 2
	cy := v.Height / 2
	maxCircleRadius := float64(v.Width/2 - 10)
	scale := maxCircleRadius / float64(v.MaxRange)

	for _, point := range points {
		distVal := point.Dist
		if distVal > v.MaxRange {
			distVal = v.MaxRange
		}

		rad := (float64(point.Angle) + 90) * math.Pi / 180
		dist := float64(distVal) * scale

		x := cx + int(dist*math.Cos(rad))
		y := cy + int(dist*math.Sin(rad))

		gocv.Circle(&img, image.Pt(x, y), 2, color.RGBA{0, 0, 255, 0}, -1)
		pointsData = append(pointsData, image.Point{x, y})
	}
	return img, pointsData
}