package main

import (
	"LIDAR/LIDAR_DRIVER"
	"LIDAR/LIDAR_VIZUALIZER"
	"fmt"
	"gocv.io/x/gocv"
	"sync"
	"sync/atomic"
	"time"
)

var points360 [360]lidardriver.ResultPoint
var mu sync.Mutex

var lidarFPS uint64
var renderFPS uint64

func main() {
	lidar, err := lidardriver.NewLD06Driver("/dev/ttyUSB0")
	if err != nil {
		panic(err)
	}

	vizulizer := lidarvizualizer.NewVizulizerLD06(
		700,
		700,
		&mu,
		&points360,
		2,
	)

	go func() {
		t := time.NewTicker(time.Second)
		for range t.C {
			fmt.Printf("LIDAR FPS: %d | Render FPS: %d\n",
				atomic.SwapUint64(&lidarFPS, 0),
				atomic.SwapUint64(&renderFPS, 0),
			)
		}
	}()

	go func() {
		for {
			results, err := lidar.ReadData()
			if err != nil {
				continue
			}

			atomic.AddUint64(&lidarFPS, 1)

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
		window := gocv.NewWindow("Lidar")
		windowMap := gocv.NewWindow("Map")

		for {
			img, pointsData := vizulizer.GetVizuliz()
			imgMap := vizulizer.FormatMap(img, pointsData, 5.0)

			atomic.AddUint64(&renderFPS, 1)
			// window.ResizeWindow(vizulizer.Width, vizulizer.Height)
			// windowMap.ResizeWindow(vizulizer.Width, vizulizer.Height)
			window.IMShow(img)
			windowMap.IMShow(imgMap)

			img.Close()
			imgMap.Close()
			gocv.WaitKey(1)
		}
	}()

	select {}
}
