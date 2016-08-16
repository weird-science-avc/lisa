package main

import (
	"fmt"
	"log"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
)

const maxSpeed = 10.0

var ErrSpeedOutOfRange = fmt.Errorf("speed must be between 0.0 to %0.1f", maxSpeed)

// ESCDriver represents a driver of an ESC (works like a Servo)
type ESCDriver struct {
	name       string
	pin        string
	connection gpio.ServoWriter
	gobot.Commander
	CurrentSpeed float64
}

// NewESCDriver creates a ESC driver given a servo writer and a pin.
func NewESCDriver(a gpio.ServoWriter, name string, pin string) *ESCDriver {
	s := &ESCDriver{
		name:         name,
		connection:   a,
		pin:          pin,
		Commander:    gobot.NewCommander(),
		CurrentSpeed: 0.0,
	}

	s.AddCommand("Speed", func(params map[string]interface{}) interface{} {
		speed := params["speed"].(float64)
		return s.Speed(speed)
	})
	return s
}

func (e *ESCDriver) Name() string { return e.name }
func (e *ESCDriver) Pin() string  { return e.pin }

func (e *ESCDriver) Connection() gobot.Connection { return e.connection.(gobot.Connection) }
func (e *ESCDriver) Start() (errs []error)        { return }
func (e *ESCDriver) Halt() (errs []error)         { return }

// Speed sets the speed for the ESC. Acceptable speeds are 0-10.0 mph.
func (e *ESCDriver) Speed(speed float64) (err error) {
	if speed < 0 || speed > maxSpeed {
		return ErrSpeedOutOfRange
	}
	e.CurrentSpeed = speed
	value := byte((speed / maxSpeed) * 180)
	log.Printf("speed: %0.3fmph => %d", speed, value)
	return e.connection.ServoWrite(e.Pin(), value)
}

//// CALIBRATION
//
//const SpeedMinServo = 660  // slowest non-zero
//const SpeedMaxServo = 1080 // fastest non-zero
//
//const SpeedMServoFromVelocity = 95.964
//const SpeedBServoFromVelocity = 659.19
//
//const SpeedStoppedServo = 0
//
//// NOTE: We can directly set velocity and servo instead of curve evaluation if we like
//const SpeedLowVelocity = 1.0
//
//var SpeedLowServo = byte(math.Max(math.Min(SpeedMServoFromVelocity*SpeedLowVelocity+SpeedBServoFromVelocity, SpeedMaxServo), SpeedMinServo))
//
//// NOTE(ppg): If you bump up the top speed you should also make it do updates in the main loop faster or allow more IMU angle variation, otherwise we turn to fast to track
//const SpeedHighVelocity = 3.5
//
//// SpeedHighServo is the maximum value for the ESC servo.
//var SpeedHighServo = byte(math.Max(math.Min(SpeedMServoFromVelocity*SpeedHighVelocity+SpeedBServoFromVelocity, SpeedMaxServo), SpeedMinServo))
