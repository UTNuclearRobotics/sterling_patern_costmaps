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
	@echo "Added docker xhost permissions"
	docker compose -f ./docker/docker-compose.yaml up -d --remove-orphans

stop:
	@echo "Stopping Docker container..."
	@docker compose -f ./docker/docker-compose.yaml down

shell:
	@echo "Opening a shell for sterling_patern_costmaps_c..."
	@docker exec -ti sterling_patern_costmaps_c bash -l

build:
	@if [ "$$(docker ps -q -f name=sterling_patern_costmaps_c)" ]; then \
		echo "Stopping container sterling_patern_costmaps_c..."; \
		docker compose -f ./docker/docker-compose.yaml down; \
	fi
	@echo "Building Docker container..."
	@docker compose -f ./docker/docker-compose.yaml build
