package main

import (
	"errors"
	"log"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
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
	// TODO(ppg): allow configuration of these values
	period := 20e6                            // (20ms)
	duty := int(600e3 + speed*(1100e3-600e3)) // 600us min, 1100us max
	log.Printf("speed: %0.3f => PwmDirectWrite(%d, %d) (%dus @ %dHz)", speed, int(period), duty/1e3, duty/1e3, int((1e9 / period)))
	return e.connection.PwmDirectWrite(e.Pin(), int(period), duty)
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
