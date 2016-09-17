package main

import (
	"fmt"
	"strings"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/beaglebone"
	"github.com/hybridgroup/gobot/platforms/firmata"
	"github.com/hybridgroup/gobot/platforms/gpio"
	"github.com/hybridgroup/gobot/platforms/intel-iot/edison"
	// TODO(ppg): Move this to gopkg.in
	"github.com/ppg/rosgo/msgs/std_msgs"
	"github.com/ppg/rosgo/ros"
)

// TODO(ppg): need to fix ROS to allow global logger more akin to C++, and to
// pipe information over the ROS bus so it can be collected by rqt_console

//func callback(msg *msgs.Hello, event ros.MessageEvent) {
//	fmt.Printf("Received: %s from %s, header = %v, time = %v\n",
//}
//		msg.Data, event.PublisherName, event.ConnectionHeader, event.ReceiptTime)

type Steerer interface {
	Steer(float64) error
}

type Speeder interface {
	Speed(float64) error
}

// Beaglebone working PWM pins:
//   P8_13
//   P9_14
//   P9_21
//   P9_42
// Not working
//   P8_34
//   P8_45
//   P8_46
//   P9_22
//   P9_29

func main() {
	node := ros.NewNode("/drivetrain")
	defer node.Shutdown()
	node.Logger().Info("starting /drivetrain")

	// TODO(ppg): consider spinning until these parameters are available;
	// also consider supporting live reloading.
	pSteeringConfig, err := node.GetParam("/drivetrain/steering")
	if err != nil {
		panic(fmt.Sprintf("failed to get steering parameter: %s", err))
	}
	steeringConfig := pSteeringConfig.(map[string]interface{})

	pSpeedConfig, err := node.GetParam("/drivetrain/speed")
	if err != nil {
		panic(fmt.Sprintf("failed to get speed parameter: %s", err))
	}
	speedConfig := pSpeedConfig.(map[string]interface{})

	// Create a robot
	robot := gobot.NewRobot("drivetrain")

	// Setup steering
	var steeringWriter gpio.ServoWriter
	switch steeringConfig["type"] {
	case "firmata":
		steeringWriter = findOrCreateFirmataAdaptor(robot, steeringConfig["port"].(string))
	case "beaglebone":
		steeringWriter = findOrCreateBeagleboneAdaptor(robot)
	case "edison":
		steeringWriter = findOrCreateEdisonAdaptor(robot)
	default:
		panic(fmt.Sprintf("Unrecognized steering device type: %s", steeringConfig["type"].(string)))
	}
	steeringDriver := NewSteeringDriver(steeringWriter, "steering", steeringConfig["pin"].(string))
	robot.AddDevice(steeringDriver)
	var steering Steerer = steeringDriver

	// Setup speed
	var speedWriter gpio.PwmDirectWriter
	switch speedConfig["type"] {
	case "firmata":
		speedWriter = findOrCreateFirmataAdaptor(robot, speedConfig["port"].(string))
	case "beaglebone":
		speedWriter = findOrCreateBeagleboneAdaptor(robot)
	case "edison":
		speedWriter = findOrCreateEdisonAdaptor(robot)
	default:
		panic(fmt.Sprintf("Unrecognized speed device type: %s", speedConfig["type"]))
	}
	ecsDriver := NewESCDriver(speedWriter, "speed", speedConfig["pin"].(string))
	robot.AddDevice(ecsDriver)
	var speed Speeder = ecsDriver

	// Start gotot Robot; stop on exit
	errs := robot.Start()
	// TODO(ppg): if we can't connect should we do something else here?
	if len(errs) > 0 {
		var msgs []string
		for _, err := range errs {
			msgs = append(msgs, err.Error())
		}
		panic(strings.Join(msgs, ", "))
	}
	defer robot.Stop()

	// Subscribe to lisa/cmd_steering and lisa/cmd_speed
	node.Logger().Info("subscribing to /lisa/cmd_steering")
	node.NewSubscriber("/lisa/cmd_steering", std_msgs.MsgFloat64, func(msg *std_msgs.Float64) {
		if err := steering.Steer(msg.Data); err != nil {
			node.Logger().Error(err.Error())
		}
	})
	node.Logger().Info("subscribing to /lisa/cmd_speed")
	node.NewSubscriber("/lisa/cmd_speed", std_msgs.MsgFloat64, func(msg *std_msgs.Float64) {
		if err := speed.Speed(msg.Data); err != nil {
			node.Logger().Error(err.Error())
		}
	})
	node.Logger().Info("spinning")
	node.Spin()
}

var firmataAdaptors map[string]*firmata.FirmataAdaptor

func findOrCreateFirmataAdaptor(robot *gobot.Robot, port string) (adaptor *firmata.FirmataAdaptor) {
	if firmataAdaptors == nil {
		firmataAdaptors = make(map[string]*firmata.FirmataAdaptor)
	}
	var ok bool
	adaptor, ok = firmataAdaptors[port]
	if !ok {
		adaptor = firmata.NewFirmataAdaptor(port, port)
		robot.AddConnection(adaptor)
		firmataAdaptors[port] = adaptor
	}
	return
}

var beagleboneAdaptor *beaglebone.BeagleboneAdaptor

func findOrCreateBeagleboneAdaptor(robot *gobot.Robot) *beaglebone.BeagleboneAdaptor {
	if beagleboneAdaptor == nil {
		beagleboneAdaptor = beaglebone.NewBeagleboneAdaptor("beaglebone")
		robot.AddConnection(beagleboneAdaptor)
	}
	return beagleboneAdaptor
}

var edisonAdaptor *edison.EdisonAdaptor

func findOrCreateEdisonAdaptor(robot *gobot.Robot) *edison.EdisonAdaptor {
	if edisonAdaptor == nil {
		edisonAdaptor = edison.NewEdisonAdaptor("edison")
		robot.AddConnection(edisonAdaptor)
	}
	return edisonAdaptor
}
