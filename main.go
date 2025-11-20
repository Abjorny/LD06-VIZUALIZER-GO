package main

import (
	"LIDAR/LIDAR_DRIVER"
	"LIDAR/LIDAR_VIZUALIZER"
	"gocv.io/x/gocv"
	"sync"
)

var points360 [360]lidardriver.ResultPoint
var mu sync.Mutex
	
func main() {
	lidar, err := lidardriver.NewLD06Driver("/dev/ttyUSB0")
	vizulizer := lidarvizualizer.NewVizulizerLD06(
		700,
		700,
		&mu,
		&points360,
		2,
	)
	if err != nil {
		panic(err)
	}

	go func() {
		for {
			results, err := lidar.ReadData()
			if err != nil {
				continue
			}
			mu.Lock()
			for _, p := range results {
				if p.Angle >= 0 && p.Angle < 360 {
					points360[p.Angle] = p
				}
			}
			mu.Unlock()
		}
	}()

	go func() {
		window := gocv.NewWindow("Circle Example")

		for {
			img := vizulizer.GetVizuliz()
			window.ResizeWindow(vizulizer.Width, vizulizer.Height)
			window.IMShow(img)
			img.Close()
			gocv.WaitKey(1)
		}
	}()
	select {}
}
