package main

import (
	"log"

	"github.com/akio/rosgo/src/ros"

	_ "std_msgs"
)

func callback(msg *std_msgs.Float64) {
	log.Printf("Received: %s\n", msg.Data)
}

func main() {
	node := ros.NewNode("/steering")
	defer node.Shutdown()
	node.Logger().SetSeverity(ros.LogLevelDebug)
	node.NewSubscriber("/lisa/cmd_steering", &MsgHello{}, callback)
	node.Spin()
}

//func main() {
//	gbot := gobot.NewGobot()
//
//	adaptor := beaglebone.NewBeagleboneAdaptor("beaglebone")
//	//adaptor := firmata.NewFirmataAdaptor("myFirmata", "/dev/ttyACM0")
//
//	button := gpio.NewButtonDriver(adaptor, "myButton", "P8_12")
//	servo := gpio.NewServoDriver(adaptor, "servo", "P8_13")
//	//led := gpio.NewLedDriver(adaptor, "myLed", "13")
//
//	work := func() {
//		gobot.On(button.Event("push"), func(data interface{}) {
//			log.Println("Button pushed")
//			//led.On()
//		})
//		gobot.On(button.Event("release"), func(data interface{}) {
//			log.Println("Button released")
//			//led.Off()
//		})
//		gobot.Every(1*time.Second, func() {
//			i := uint8(gobot.Rand(180))
//			fmt.Println("Turning", i)
//			servo.Move(i)
//		})
//	}
//
//	robot := gobot.NewRobot("buttonBot",
//		[]gobot.Connection{adaptor},
//		[]gobot.Device{button, servo}, //, led},
//		work,
//	)
//
//	gbot.AddRobot(robot)
//	gbot.Start()
//}
