package main

import (
	"errors"
	"log"
	"math"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
)

// TODO(ppg): consider renaming to AckermanDriver

const (
	// Found by experimentation, and deviation is
	//steeringServoMin    = 40 // limit
	//steeringServoMax    = 125 // limit
	steeringServoCenter = 82
	steeringServoMin    = 50  // backed off some
	steeringServoMax    = 115 // backed off some
)

var (
	steeringServoRange = byte(math.Min(steeringServoMax-steeringServoCenter, steeringServoCenter-steeringServoMin))

	ErrSteeringOutOfRange = errors.New("steering must be between -1.0 to 1.0")
)

// SteeringDriver represents a driver of an ESC (works like a Servo)
type SteeringDriver struct {
	name       string
	pin        string
	connection gpio.ServoWriter
	gobot.Commander
	CurrentSteering float64
}

// NewSteeringDriver creates a ESC driver given a servo writer and a pin.
func NewSteeringDriver(a gpio.ServoWriter, name string, pin string) *SteeringDriver {
	s := &SteeringDriver{
		name:            name,
		connection:      a,
		pin:             pin,
		Commander:       gobot.NewCommander(),
		CurrentSteering: 0.0,
	}

	s.AddCommand("Steer", func(params map[string]interface{}) interface{} {
		steering := params["steering"].(float64)
		return s.Steer(steering)
	})
	return s
}

func (s *SteeringDriver) Name() string { return s.name }
func (s *SteeringDriver) Pin() string  { return s.pin }

func (s *SteeringDriver) Connection() gobot.Connection { return s.connection.(gobot.Connection) }
func (s *SteeringDriver) Start() (errs []error)        { return }
func (s *SteeringDriver) Halt() (errs []error)         { return }

// Steer sets the steering for the Ackerman system. Acceptable steering is -1.0-1.0.
func (s *SteeringDriver) Steer(steering float64) (err error) {
	if steering < -1.0 || steering > 1.0 {
		return ErrSteeringOutOfRange
	}
	s.CurrentSteering = steering

	// Calculate steering based on center offset for range
	value := byte(float64(steeringServoCenter) + float64(steeringServoRange)*steering)
	log.Printf("steering: %0.3f => ServoWrite(%d)", steering, value)
	return s.connection.ServoWrite(s.Pin(), value)
}
