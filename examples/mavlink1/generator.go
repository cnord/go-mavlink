package mavlink

//go:generate mavgen -f ../../mavlink-upstream/message_definitions/v1.0/common.xml -m 1 -p
//go:generate mavgen -f ../../mavlink-upstream/message_definitions/v1.0/ardupilotmega.xml -m 1
//go:generate mavgen -f ../../mavlink-upstream/message_definitions/v1.0/ASLUAV.xml -m 1
//go:generate mavgen -f ../../mavlink-upstream/message_definitions/v1.0/matrixpilot.xml -m 1
//go:generate mavgen -f ../../mavlink-upstream/message_definitions/v1.0/ualberta.xml -m 1

func main() {
}
