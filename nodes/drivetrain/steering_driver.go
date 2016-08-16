package main

import (
	"fmt"
	"log"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
)

// TODO(ppg): consider renaming to AckermanDriver

const (
	minSteering = -1.0
	maxSteering = 1.0
)

var ErrSteeringOutOfRange = fmt.Errorf("steering must be between %0.1f to %0.1f", minSteering, maxSteering)

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
	if steering < minSteering || steering > maxSteering {
		return ErrSteeringOutOfRange
	}
	s.CurrentSteering = steering
	value := byte((steering - minSteering) / (maxSteering - minSteering) * 180)
	log.Printf("steering: %0.3f => %d", steering, value)
	return s.connection.ServoWrite(s.Pin(), value)
}
