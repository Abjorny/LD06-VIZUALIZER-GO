package main

import (
	"LIDAR/LIDAR_DRIVER"
	"LIDAR/LIDAR_VIZUALIZER"
	_"fmt"
	"gocv.io/x/gocv"
	"sync"
	"sync/atomic"
	"time"
	"os"
	"fmt"
	// "net/http"
	// "github.com/gin-gonic/gin"
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
			// fmt.Printf("LIDAR FPS: %d | Render FPS: %d\n",
			// 	atomic.SwapUint64(&lidarFPS, 0),
			// 	atomic.SwapUint64(&renderFPS, 0),
			// )
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
	imgMap := vizulizer.FormatMap(img, pointsData, 0.5)

	atomic.AddUint64(&renderFPS, 1)
	window.ResizeWindow(vizulizer.Width, vizulizer.Height)
	windowMap.ResizeWindow(vizulizer.Width, vizulizer.Height)
	window.IMShow(img)
	windowMap.IMShow(imgMap)

	key := gocv.WaitKey(1)
	if key == 's' || key == 'S' {
		fileName := "points_data.txt"
		f, err := os.Create(fileName)
		if err != nil {
			fmt.Println("Ошибка при создании файла:", err)
		} else {
			for _, p := range points360 {
				fmt.Fprintf(f, "%f\n", p.Dist)
			}
			fmt.Println("pointsData сохранён в", fileName)
		}
	}

	img.Close()
	imgMap.Close()
}

	}()

	// go func() {
	// 	router := gin.Default()
	// 	router.GET("/ping", func(c *gin.Context){
	// 		c.JSON(http.StatusOK, gin.H{
	// 			"message": "pong",
	// 		})
	// 	})
	// 	router.Run()
	// } ()
	select {}
}
