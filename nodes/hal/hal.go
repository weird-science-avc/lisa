package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"io/ioutil"

	// TODO(ppg): Move this to gopkg.in
	"github.com/ppg/rosgo/msgs/geometry_msgs"
	"github.com/ppg/rosgo/msgs/std_msgs"
	"github.com/ppg/rosgo/ros"
	"github.com/ppg/rosgo/srvs/std_srvs"
)

const (
	clearWaypointsRPC         = "/waypoint_manager/clear"
	resetWaypointsRPC         = "/waypoint_manager/reset"
	startNavRPC               = "/navigator/start"
	initialPoseTopic          = "initialpose"
	poseTopic                 = "/move_base_simple/goal"
	defaultCourseMapParameter = "/hal/map_location"
	defaultNodeName           = "hal9000"
)

var zeroZeroCoordinate = &geometry_msgs.PoseWithCovarianceStamped{}

type JsonCoordinate struct {
	X         int
	Y         int
	Tolerance float64
}

type Hal9000 struct {
	course               []*geometry_msgs.PoseStamped
	node                 ros.Node
	rpcCalls             map[string]ros.ServiceClient
	initialPositionTopic ros.Publisher
	positionTopic        ros.Publisher
}

func NewHal9000() *Hal9000 {
	return &Hal9000{
		rpcCalls: map[string]ros.ServiceClient{},
	}
}

func (hal *Hal9000) Init() error {
	hal.node = ros.NewNode(defaultNodeName)
	hal.node.Logger().Info("starting /hal")
	hal.rpcCalls["clearWaypoints"] = hal.node.NewServiceClient(clearWaypointsRPC, std_srvs.SrvEmpty)
	hal.rpcCalls["resetWaypoints"] = hal.node.NewServiceClient(resetWaypointsRPC, std_srvs.SrvEmpty)
	hal.rpcCalls["startNavigation"] = hal.node.NewServiceClient(startNavRPC, std_srvs.SrvEmpty)

	hal.initialPositionTopic = hal.node.NewPublisherWithCallbacks(initialPoseTopic, geometry_msgs.MsgPoseWithCovarianceStamped,
		func(ros.SingleSubscriberPublisher) {
			hal.node.Logger().Infof("Publishing on %s", initialPoseTopic)
		},
		func(ros.SingleSubscriberPublisher) {
			hal.node.Logger().Errorf("Lost connection with %s", initialPoseTopic)
		},
	)

	hal.positionTopic = hal.node.NewPublisherWithCallbacks(poseTopic, geometry_msgs.MsgPoseStamped,
		func(ros.SingleSubscriberPublisher) {
			hal.node.Logger().Infof("Publishing on %s", initialPoseTopic)
		},
		func(ros.SingleSubscriberPublisher) {
			hal.node.Logger().Errorf("Lost connection with %s", initialPoseTopic)
		},
	)
	return nil
}

func (hal *Hal9000) Shutdown() {
	hal.node.Logger().Info("shutting down /hal")
	hal.shutdownRPC()
	hal.node.Shutdown()
}

func (hal *Hal9000) shutdownRPC() {
	for _, call := range hal.rpcCalls {
		call.Shutdown()
	}
}

func (hal *Hal9000) Clear() {
	hal.rpcCalls["clearWaypoints"].Call(&std_srvs.Empty{})
}

func (hal *Hal9000) CalibrateInitialPosition() {
	hal.initialPositionTopic.Publish(zeroZeroCoordinate)
}

func (hal *Hal9000) Reset() {
	hal.rpcCalls["resetWaypoints"].Call(&std_srvs.Empty{})
}

func (hal *Hal9000) Start() {
	hal.rpcCalls["startNavigation"].Call(&std_srvs.Empty{})
}

func (hal *Hal9000) LoadCourse(courseMapParameter string) error {
	hal.node.Logger().Info("loading course map")
	courseLocation, err := hal.node.GetParam(courseMapParameter)
	if (err != nil) || (courseLocation == nil) {
		return errors.New(fmt.Sprintf("Failed to read map_location parameter (%s)", err))
	}

	// Read map from mapLocation
	jsonWaypoints, err := ioutil.ReadFile(courseLocation.(string))
	if err != nil {
		return errors.New(fmt.Sprintf("Failed to read course map from map_location (%s)", err))
	}

	hal.course, err = hal.ParseCoordinatesFromJSONString(jsonWaypoints)
	if err != nil {
		return errors.New(fmt.Sprintf("Failed to parse coordinates from map (%s)", err))
	}

	return nil
}

func (hal *Hal9000) ParseCoordinatesFromJSONString(raw []byte) ([]*geometry_msgs.PoseStamped, error) {
	coords := []*geometry_msgs.PoseStamped{}
	jsonCoords := []JsonCoordinate{}

	var stringMap map[string]*json.RawMessage
	err := json.Unmarshal(raw, &stringMap)
	if err != nil {
		return coords, err
	}

	err = json.Unmarshal(*stringMap["waypoints"], &jsonCoords)
	if err != nil {
		return coords, err
	}

	for _, coord := range jsonCoords {
		coords = append(coords, &geometry_msgs.PoseStamped{
			Header: std_msgs.Header{},
			Pose: geometry_msgs.Pose{
				Position: geometry_msgs.Point{
					X: float64(coord.X),
					Y: float64(coord.Y),
					Z: 0.0,
				},
			},
		})
	}

	return coords, nil
}

func (hal *Hal9000) PublishWaypoint(waypoint *geometry_msgs.PoseStamped) error {
	hal.positionTopic.Publish(waypoint)
	hal.node.Logger().Infof("%s", waypoint)
	return nil
}

func (hal *Hal9000) PublishCourse() error {
	for _, waypoint := range hal.course {
		err := hal.PublishWaypoint(waypoint)
		if err != nil {
			return err
		}
	}
	return nil
}

func (hal *Hal9000) StartNavigation() error {
	for _, waypoint := range hal.course {
		err := hal.PublishWaypoint(waypoint)
		if err != nil {
			return err
		}
	}
	return nil
}

func main() {
	selfAware := NewHal9000()

	err := selfAware.Init()
	if err != nil {
		panic(fmt.Sprintf("Failed to initialize node: %s", err))
	}
	defer selfAware.Shutdown()

	// Button press triggers here
	err = selfAware.LoadCourse(defaultCourseMapParameter)
	if err != nil {
		panic(fmt.Sprintf("Error loading course map: %s", err))
	}

	selfAware.Clear()
	err = selfAware.PublishCourse()
	if err != nil {
		selfAware.node.Logger().Errorf("Error publishing course: %s", err)
	}
	selfAware.Reset()
	selfAware.CalibrateInitialPosition()
	selfAware.Start()
}
