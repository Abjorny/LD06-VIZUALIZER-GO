package main

import (
	"encoding/binary"
	"log"
	"net/http"
	"sync"
	lidardriver "LIDAR/LIDAR_DRIVER"
	"github.com/gorilla/websocket"
)

var points360 [360]lidardriver.ResultPoint
var mu sync.Mutex

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

var clients = make(map[*websocket.Conn]bool)
var clientsMu sync.Mutex

func handleConnections(w http.ResponseWriter, r *http.Request) {
	ws, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer ws.Close()

	clientsMu.Lock()
	clients[ws] = true
	clientsMu.Unlock()

	for {
		if _, _, err := ws.NextReader(); err != nil {
			break
		}
	}

	clientsMu.Lock()
	delete(clients, ws)
	clientsMu.Unlock()
}

func broadcastPoints() {
	buf := make([]byte, 360*5)
	mu.Lock()
	for i, p := range points360 {
		binary.LittleEndian.PutUint16(buf[i*5:], uint16(p.Dist*1000))
		buf[i*5+2] = byte(p.Intes)
		binary.LittleEndian.PutUint16(buf[i*5+3:], uint16(p.Angle))
	}
	mu.Unlock()

	clientsMu.Lock()
	for ws := range clients {
		if err := ws.WriteMessage(websocket.BinaryMessage, buf); err != nil {
			log.Println("ws write error:", err)
			ws.Close()
			delete(clients, ws)
		}
	}
	clientsMu.Unlock()
}

func main() {
	lidar, err := lidardriver.NewLD06Driver("/dev/ttyUSB0")
	if err != nil {
		panic(err)
	}

	http.HandleFunc("/ws", handleConnections)
	go func() {
		log.Println("WebSocket server on :8000/ws")
		log.Fatal(http.ListenAndServe(":8000", nil))
	}()

	for {
		results, err := lidar.ReadData()
		if err != nil {
			continue
		}
		mu.Lock()
		for _, p := range results {
			angle := int(p.Angle) % 360
			points360[angle] = p
		}
		mu.Unlock()
		broadcastPoints()
	}
}
