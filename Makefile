help:
	@echo "Usage:"
	@echo "make start            - start the container"
	@echo "make stop             - stop the container"
	@echo "make shell            - open a shell in the container"
	@echo "make build            - build the container image"
	@echo ""

start:
	@echo "Starting Docker container..."
	@xhost +si:localuser:root
	@echo "added docker xhost permissions"
	@export UID_GID=$$(id -u):$$(id -g); \
	export UNAME=$$(whoami); \
	echo "gathering host user info... $$UNAME $$UID_GID"; \
	docker compose -f ./docker/docker-compose.yaml up -d

stop:
	@echo "Stopping Docker container..."
	@docker compose -f ./docker/docker-compose.yaml down

shell:
	@echo "Opening a shell for patern_gazebo_c..."
	@docker exec -ti patern_gazebo_c bash -l

build:
	@if [ "$$(docker ps -q -f name=patern_gazebo_c)" ]; then \
		echo "stopping container patern_gazebo_c..."; \
		docker compose -f ./docker/docker-compose.yaml down; \
	fi
	@echo "Building Docker container..."
	@docker compose -f ./docker/docker-compose.yaml build
