#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera
{
public:
	Camera(glm::vec3 position, glm::vec3 target, glm::vec3 worldup);
	Camera(glm::vec3 position, float pitch, float yaw, glm::vec3 worldup);
	~Camera();

	glm::mat4 GetViewMatix();
	void ProcessMouseMovement(float deltaX, float deltaY);

	float moveX = 0;
	float moveY = 0;
	float moveZ = 0;
	void updateCameraPos();
private:

	glm::vec3 Position;
	glm::vec3 Forward;
	glm::vec3 Right;
	glm::vec3 Up;
	glm::vec3 WorldUp;
	float Pitch;
	float Yaw;

	void updateCameraVectors();

};

