#include "main/samples/06_move_into.h"

#include "neko/core/profile.h"

namespace neko
{

constexpr static Vec2f kGroundHalfSize = Vec2f{ Scalar{ 5 }, Scalar{ 1 }};
constexpr static Vec2f kRectHalfSize = Vec2f{ Ratio(1, 2), Ratio(1, 2) };
constexpr static ColorId kGroundColor = sample_color::kGreen;
constexpr static ColorId kInAirColor = sample_color::kBlue;
constexpr static ColorId kOnGroundColor = sample_color::kMagenta;
constexpr static auto kMaxForce = Scalar{ 10 };

#ifndef NEKO_SAMPLE_BOX2D
void MoveIntoSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p)
{
}

void MoveIntoSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p)
{
}

void MoveIntoSample::OnCollisionEnter(const ColliderPair& p)
{
	ColliderPair static_pair{static_collider_index_, ground_collider_index_};
	ColliderPair dynamic_pair{dynamic_collider_index_, ground_collider_index_};

	if(p == static_pair)
	{
		static_body_on_ground_ = true;
	}
	if(p == dynamic_pair)
	{
		dynamic_body_on_ground_ = true;
	}
}

void MoveIntoSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
}
#else
/// Called when two fixtures begin to touch.
void MoveIntoSample::BeginContact(b2Contact* contact)
{
	const auto user_data_a = contact->GetFixtureA()->GetUserData().pointer;
	const auto user_data_b = contact->GetFixtureB()->GetUserData().pointer;

	if((b2Body*)user_data_a == b2_ground_body_ || (b2Body*)user_data_b == b2_ground_body_)
	{
		if((b2Body*)user_data_a == b2_static_body_ || (b2Body*)user_data_b == b2_static_body_)
		{
			static_body_on_ground_ = true;
		}

		if((b2Body*)user_data_a == b2_dynamic_body_ || (b2Body*)user_data_b == b2_dynamic_body_)
		{
			dynamic_body_on_ground_ = true;
		}
	}
}

/// Called when two fixtures cease to touch.
void MoveIntoSample::EndContact(b2Contact* contact)
{
}
#endif
void MoveIntoSample::Begin(const SampleContext& context)
{
	const auto world_center = context.world_bounds.GetCenter();
	const auto ground_position = world_center + Vec2f{ Scalar{ 0 }, Scalar{ 3 } };
	const auto dynamic_rect_position = world_center - Vec2f{ Scalar{ -2 }, Scalar{ 3 } };
	const auto static_rect_position = world_center - Vec2f{ Scalar{ 2 }, Scalar{ 3 } };
#ifndef NEKO_SAMPLE_BOX2D
	world_.SetContactListener(this);
	world_.SetBSH(&quad_tree_);
#else
	b2_world_.SetContactListener(this);
#endif
	//Implement ground
	{
#ifndef NEKO_SAMPLE_BOX2D
		ground_body_index_ = world_.AddBody();
		Body& body = world_.body(ground_body_index_);
		body.position = ground_position;
		body.velocity = Vec2f{};
		body.type = BodyType::kStatic;
		body.inverse_mass = Scalar{ 0 };

		ground_collider_index_ = world_.AddAabbCollider(ground_body_index_);
		auto& collider = world_.collider(ground_collider_index_);
		auto& aabb_collider = world_.aabb(collider.shape_index);
		aabb_collider.half_size = kGroundHalfSize;
		collider.is_trigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef body_def{};
		body_def.position = b2Vec2{ float{ground_position.x()}, float{ground_position.y()} };
		body_def.type = b2_staticBody;
		body_def.fixedRotation = true;
		b2_ground_body_ = b2_world_.CreateBody(&body_def);

		b2PolygonShape shape;
		shape.SetAsBox(float{kGroundHalfSize.x()}, float{kGroundHalfSize.y()});
		b2FixtureDef fixture_def{};
		fixture_def.shape = &shape;
		fixture_def.friction = 0.0f;
		fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(b2_ground_body_);
		b2_ground_body_->CreateFixture(&fixture_def);
#endif
	}
	{
#ifndef NEKO_SAMPLE_BOX2D
		//rect ball
		static_body_index_ = world_.AddBody();
		Body& body = world_.body(static_body_index_);
		body.position = static_rect_position;
		body.velocity = Vec2f{};
		body.type = BodyType::kDynamic;
		body.inverse_mass = Scalar{ 1 };

		static_collider_index_ = world_.AddAabbCollider(static_body_index_);
		auto& collider = world_.collider(static_collider_index_);
		auto& aabb_collider = world_.aabb(collider.shape_index);
		aabb_collider.half_size = kRectHalfSize;
		collider.is_trigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef body_def{};
		body_def.position = { float{static_rect_position.x()}, float{static_rect_position.y()} };
		body_def.type = b2_dynamicBody;
		body_def.fixedRotation = true;
		b2_static_body_ = b2_world_.CreateBody(&body_def);

		b2PolygonShape shape{};
		shape.SetAsBox(float{kRectHalfSize.x()}, float{kRectHalfSize.y()});

		b2FixtureDef fixture_def{};
		fixture_def.shape = &shape;
		fixture_def.friction = 0.0f;
		fixture_def.density = 1.0f;
		fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(b2_static_body_);
		b2_static_body_->CreateFixture(&fixture_def);
#endif
	}

	{
#ifndef NEKO_SAMPLE_BOX2D
		//rect ball
		dynamic_body_index_ = world_.AddBody();
		Body& body = world_.body(dynamic_body_index_);
		body.position = dynamic_rect_position;
		body.velocity = Vec2f{};
		body.type = BodyType::kDynamic;
		body.inverse_mass = Scalar{ 1 };

		dynamic_collider_index_ = world_.AddAabbCollider(dynamic_body_index_);
		auto& collider = world_.collider(dynamic_collider_index_);
		auto& aabb_collider = world_.aabb(collider.shape_index);
		aabb_collider.half_size = kRectHalfSize;
		collider.is_trigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef body_def{};
		body_def.position = { float{dynamic_rect_position.x()}, float{dynamic_rect_position.y()} };
		body_def.type = b2_dynamicBody;
		body_def.fixedRotation = true;
		b2_dynamic_body_ = b2_world_.CreateBody(&body_def);

		b2PolygonShape shape{};
		shape.SetAsBox(float{kRectHalfSize.x()}, float{kRectHalfSize.y()});

		b2FixtureDef fixture_def{};
		fixture_def.shape = &shape;
		fixture_def.density = 1.0f;
		fixture_def.friction = 0.0f;
		fixture_def.userData.pointer = reinterpret_cast<uintptr_t>(b2_dynamic_body_);
		b2_dynamic_body_->CreateFixture(&fixture_def);
#endif
	}
}

void MoveIntoSample::Draw(DrawList& draw_list) const
{
#ifdef NEKO_SAMPLE_BOX2D
	const auto position = [](const b2Body* body)
	{
		const auto pos = body->GetPosition();
		return Vec2f{ Scalar{ pos.x }, Scalar{ pos.y } };
	};
	const Vec2f ground_center = position(b2_ground_body_);
	const Vec2f static_position = position(b2_static_body_);
	const Vec2f dynamic_position = position(b2_dynamic_body_);
#else
	const Vec2f ground_center = world_.body(ground_body_index_).position;
	const Vec2f static_position = world_.body(static_body_index_).position;
	const Vec2f dynamic_position = world_.body(dynamic_body_index_).position;
#endif
	draw_list.boxes.push_back({ ground_center, kGroundHalfSize, kGroundColor });
	draw_list.boxes.push_back({ static_position, kRectHalfSize, static_body_on_ground_ ? kOnGroundColor : kInAirColor });
	draw_list.boxes.push_back({ dynamic_position, kRectHalfSize, dynamic_body_on_ground_ ? kOnGroundColor : kInAirColor });
}

void MoveIntoSample::End()
{
#ifdef NEKO_SAMPLE_BOX2D
	b2_world_ = b2World{{0.0f, -9.81f}};
#else
	world_.Clear();
	quad_tree_.Clear();
#endif
	static_body_on_ground_ = false;
	dynamic_body_on_ground_ = false;
}

std::uint32_t MoveIntoSample::GenerateChecksum() const
{
#ifdef NEKO_SAMPLE_BOX2D
	// Box2D isn't checked for determinism
	return 0;
#else
	return world_.GenerateChecksum();
#endif
}

void MoveIntoSample::FixedUpdate()
{
	NEKO_PROFILE_FUNCTION();

	if (dynamic_body_on_ground_)
	{
#ifdef NEKO_SAMPLE_BOX2D
		const Scalar mass = (Scalar)b2_dynamic_body_->GetMass();
		const Vec2f velocity {b2_dynamic_body_->GetLinearVelocity()};
		constexpr neko::Vec2f wanted_velocity{ Scalar{ -1 }, Scalar{}};
		auto force = (wanted_velocity - velocity) / kFixedDeltaTime * mass ;
		if (force.SquareLength() > kMaxForce * kMaxForce)
		{
			force = force.Normalized() * kMaxForce;
		}
		b2_dynamic_body_->ApplyForceToCenter({float{force.x()}, float{force.y()}}, true);
#else
		Body& dynamic_body = world_.body(dynamic_body_index_);
		//Move dynamic object to the right
		constexpr neko::Vec2f wanted_velocity{ Scalar{ -1 }, Scalar{}};
		auto force = (wanted_velocity - dynamic_body.velocity) / kFixedDeltaTime / dynamic_body.inverse_mass;
		if (force.SquareLength() > kMaxForce * kMaxForce)
		{
			force = force.Normalized() * kMaxForce;
		}
		dynamic_body.force += force;
#endif
	}
	if (static_body_on_ground_)
	{
#ifdef NEKO_SAMPLE_BOX2D
		const Scalar mass = (Scalar)b2_static_body_->GetMass();
		const Vec2f velocity {b2_static_body_->GetLinearVelocity()};
		constexpr neko::Vec2f wanted_velocity{};
		auto force = (wanted_velocity - velocity) / kFixedDeltaTime * mass ;
		if (force.SquareLength() > kMaxForce * kMaxForce)
		{
			force = force.Normalized() * kMaxForce;
		}
		b2_static_body_->ApplyForceToCenter({float{force.x()}, float{force.y()}}, true);
#else
		//Use force to make object static
		Body& static_body = world_.body(static_body_index_);
		//Move dynamic object to the right
		constexpr neko::Vec2f wanted_velocity{};
		auto force = (wanted_velocity - static_body.velocity) / kFixedDeltaTime / static_body.inverse_mass;
		if (force.SquareLength() > kMaxForce * kMaxForce)
		{
			force = force.Normalized() * kMaxForce;
		}
		static_body.force += force;
#endif
	}
#ifdef NEKO_SAMPLE_BOX2D
	b2_world_.Step(float{kFixedDeltaTime}, 10, 3);
#else
	world_.Step();
#endif

}
}
