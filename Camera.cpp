#include "Camera.h"
#include <iostream>

#define PI 3.1415926535898f

Camera::Camera(glm::vec3 position, glm::vec3 target, glm::vec3 worldup) {
	Position = position;
	WorldUp = worldup;
	Forward = glm::normalize(target - position);
	Right = glm::normalize(glm::cross(Forward, WorldUp));
	Up = glm::normalize(glm::cross(Right, Forward));

}

Camera::Camera(glm::vec3 position, float pitch, float yaw, glm::vec3 worldup) {
	Position = position;
	WorldUp = worldup;
	Pitch = pitch;
	Yaw = yaw;
	Forward.x = glm::cos(Pitch) * glm::sin(Yaw);
	Forward.y = glm::sin(Pitch);		   
	Forward.z = glm::cos(Pitch) * glm::cos(Yaw);
	Right = glm::normalize(glm::cross(Forward, WorldUp));
	Up = glm::normalize(glm::cross(Right, Forward));
}


Camera::~Camera() {

}

glm::mat4 Camera::GetViewMatix() {
	return glm::lookAt(Position, Position + Forward, WorldUp);

}

void Camera::ProcessMouseMovement(float deltaX, float deltaY) {
	Pitch += deltaY;
	Yaw += deltaX;

	if (Pitch > PI * 0.4f)
		Pitch = PI * 0.4f;
	if (Pitch < -PI * 0.4f)
		Pitch = -PI * 0.4f;

	updateCameraVectors();
}

void Camera::updateCameraVectors() {
	Forward.x = glm::cos(Pitch) * glm::sin(Yaw);
	Forward.y = glm::sin(Pitch);
	Forward.z = glm::cos(Pitch) * glm::cos(Yaw);
	Right = glm::normalize(glm::cross(Forward, WorldUp));
	Up = glm::normalize(glm::cross(Right, Forward));
}

void Camera::updateCameraPos() {
	Position += Forward * moveZ;
	Position += Right * moveX;
	Position += Up * moveY;
}