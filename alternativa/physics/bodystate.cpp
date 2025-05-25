#include "bodystate.h"
#include "alternativa/math/vector3.h"
#include "alternativa/math/quaternion.h"

BodyState::BodyState()
{

}

void BodyState::copy(BodyState *state)
{
	//this->posX.value = state->pos.x;
	//this->posY.value = state->pos.y;
	//this->posZ.value = state->pos.z;
	//this->velX.value = state->velocity.x;
	//this->velY.value = state->velocity.y;
	//this->velZ.value = state->velocity.z;
	//Vector3 *posSecure = new Vector3(this.posX.value, this.posY.value, this.posZ.value);
	//Vector3 *velSecure = new Vector3(this.velX.value, this.velY.value, this.velZ.value);
	this->pos.copy(&state->pos);
	this->orientation.copy(&state->orientation);
	this->velocity.copy(&state->velocity);
	this->rotation.copy(&state->rotation);
}
