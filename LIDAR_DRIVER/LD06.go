package lidardriver

import (
	"log"
	"math"
	"go.bug.st/serial"
)

type LD06Driver struct {
	port serial.Port
}

type Point struct {
	dist float64
	intes int
}

type ResultPoint struct {
	Dist float64
	Intes int
	Angle int
}

func NewLD06Driver(portName string) (*LD06Driver, error) {
	mode := &serial.Mode{
		BaudRate: 230400,
		DataBits: 8,
		StopBits: serial.OneStopBit,
		Parity:   serial.NoParity,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		log.Fatal("Ошибка открытия порта ", err)
		return nil, err
	}
	return &LD06Driver{port: port}, nil
}

func (d *LD06Driver) readPackage() ([]byte, error) {
	buf := make([]byte, 1)
	for {
		_, err := d.port.Read(buf)
		if err != nil {
			return nil, err
		}
		if buf[0] == 0x54 {
			packet := make([]byte, 47)
			packet[0] = buf[0]
			offset := 1
			for offset < 47 {
				n, err := d.port.Read(packet[offset:])
				if err != nil {
					return nil, err
				}
				offset += n
			}
			return packet, nil
		}
	}
}


func (d *LD06Driver) ReadData() (results []ResultPoint, err error){
	packet, err := d.readPackage()
	if err == nil{
		point_count := int(packet[1] & 0b00011111)
		if point_count == 12{
			start_angle := float64(int(packet[4]) + int(packet[5])<<8) / 100.0
			points := []Point{};
			offset := 6
			for i:= 0 ; i < point_count; i++{
				dist := float64(int(packet[offset]) + int(packet[offset+1])<<8) / 1000.0
				intes := int(packet[offset + 2])
				points = append(points, Point{dist, intes})
				offset += 3
			}
			end_angle := float64(int(packet[offset]) + int(packet[offset+1])<<8) / 100.0
			if end_angle < start_angle{
				end_angle += 360
			}
			step := float64(end_angle - start_angle) / float64(point_count)
			
			for i := 0; i < point_count; i++{
				angle := start_angle + step *  float64(i)
				angle = math.Mod(angle, 360)
				results = append(results, ResultPoint{
					Dist: points[i].dist,
					Intes: points[i].intes,
					Angle: int(math.Floor(angle)),
				})
			}
		}
		return 
	}
	return nil, err
}