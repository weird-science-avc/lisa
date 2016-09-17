package main

import (
	"errors"
	"log"
	"math"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
)

const (
	// Found by experimentation; values in nanoseconds
	//speedDutyArmMin       = 600e3 // limit
	//speedDutyStartMoving  = 725e3 // limit

	speedDutyStopped = 650e3
	speedDutyMin     = 800e3
	speedDutyMax     = 850e3

	// 20ms fixed by ESC
	speedPeriod = 20e6
)

var ErrSpeedOutOfRange = errors.New("speed must be between 0.0 to 1.0")

// ESCDriver represents a driver of an ESC (works like a Servo)
type ESCDriver struct {
	name       string
	pin        string
	connection gpio.PwmDirectWriter
	gobot.Commander
	CurrentSpeed float64
}

// NewESCDriver creates a ESC driver given a pwm direct writer and a pin.
// An ESC requires the full range (mainly to 0) of PWM control:
// - PwmWriter (Want 5%-10% duty cycle, limits us to about 12 steps of control)
// - ServoWriter (0 doesn't map to 0 duty cycle)
func NewESCDriver(a gpio.PwmDirectWriter, name string, pin string) *ESCDriver {
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

// Speed sets the speed for the ESC. Acceptable speeds are 0(stopped)-1.0(max) and are unitless.
func (e *ESCDriver) Speed(speed float64) (err error) {
	if speed < 0 || speed > 1.0 {
		return ErrSpeedOutOfRange
	}
	e.CurrentSpeed = speed

	// Calculate period and duty in ns
	duty := int(math.Floor(speedDutyMin + (speedDutyMax-speedDutyMin)*speed))
	// If we're at 0, really put us at our stopped duty
	if speed == 0 {
		duty = speedDutyStopped
	}
	log.Printf("speed: %0.3f => PwmDirectWrite(%d, %d) (%dus @ %0.1fHz)", speed, int(speedPeriod), int(duty), int(duty/1e3), (1e9 / speedPeriod))
	return e.connection.PwmDirectWrite(e.Pin(), speedPeriod, duty)
}
