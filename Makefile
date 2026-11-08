# Run from ~/g1_autonomy. Uses a NEW Compose project; never acts on old containers.
COMPOSE := sudo docker compose -p g1-autonomy -f docker-compose.yml
ALL := --profile unitree --profile autonomy

.DEFAULT_GOAL := help
.PHONY: help check build launch launch-unitree launch-autonomy stop status logs controller

help:
	@echo "make build            Build the thin G1 runtime image from the tested image"
	@echo "make check            Validate the Compose configuration"
	@echo "make launch           Start all services WITH live logs (not detached)"
	@echo "make launch-unitree   Start only MuJoCo and G1 controller"
	@echo "make launch-autonomy  Start only the ROS/autonomy services"
	@echo "make controller       Run G1 controller interactively"
	@echo "make status           Show this project's services"
	@echo "make logs             Follow this project's service logs"
	@echo "make stop             Stop ONLY this new Compose project"

check:
	$(COMPOSE) $(ALL) config -q

images:
	$(COMPOSE) build mujoco

build:
	$(COMPOSE) --profile build run --rm --no-deps build_g1

launch:
	$(COMPOSE) $(ALL) up --no-build --no-attach mujoco --no-attach g1_ctrl

launch-unitree:
	$(COMPOSE) --profile unitree up --no-build

launch-autonomy:
	$(COMPOSE) --profile autonomy up --no-build

stop:
	$(COMPOSE) $(ALL) down

status:
	$(COMPOSE) $(ALL) ps -a

logs:
	$(COMPOSE) $(ALL) logs -f --tail=100

controller:
	$(COMPOSE) exec --user helene:helene g1_ctrl bash -lc 'source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/opt/cyclonedds/lib:$$LD_LIBRARY_PATH && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && export ROS_DOMAIN_ID=0 && cd /workspace/unitree_rl_lab/deploy/robots/g1_29dof && exec ./build/g1_ctrl --network lo'