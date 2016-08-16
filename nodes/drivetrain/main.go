package main

import (
	"fmt"
	"strings"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/firmata"
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
	var steering Steerer
	switch steeringConfig["type"] {
	case "firmata":
		firmataAdaptor := findOrConnectFirmataAdaptor(robot, steeringConfig["port"].(string))
		d := NewSteeringDriver(firmataAdaptor, "steering", steeringConfig["pin"].(string))
		robot.AddDevice(d)
		steering = d
	default:
		panic(fmt.Sprintf("Unrecognized steering device type: %s", steeringConfig["type"].(string)))
	}

	// Setup speed
	var speed Speeder
	switch speedConfig["type"] {
	case "firmata":
		firmataAdaptor := findOrConnectFirmataAdaptor(robot, speedConfig["port"].(string))
		d := NewESCDriver(firmataAdaptor, "speed", speedConfig["pin"].(string))
		robot.AddDevice(d)
		speed = d
	default:
		panic(fmt.Sprintf("Unrecognized speed device type: %s", speedConfig["type"]))
	}

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

	// Subscribe to lisa/cmd_steering and lisa/cmd_velocity
	node.Logger().Info("subscribing to /lisa/cmd_steering")
	node.NewSubscriber("/lisa/cmd_steering", std_msgs.MsgFloat64, func(msg *std_msgs.Float64) {
		if err := steering.Steer(msg.Data); err != nil {
			node.Logger().Error(err.Error())
		}
	})
	node.Logger().Info("subscribing to /lisa/cmd_velocity")
	node.NewSubscriber("/lisa/cmd_velocity", std_msgs.MsgFloat64, func(msg *std_msgs.Float64) {
		if err := speed.Speed(msg.Data); err != nil {
			node.Logger().Error(err.Error())
		}
	})
	node.Logger().Info("spinning")
	node.Spin()
}

var firmataAdaptors map[string]*firmata.FirmataAdaptor

func findOrConnectFirmataAdaptor(robot *gobot.Robot, port string) (adaptor *firmata.FirmataAdaptor) {
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
