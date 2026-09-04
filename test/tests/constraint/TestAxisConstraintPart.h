/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef TEST_AXIS_CONSTRAINT_PART_H
#define TEST_AXIS_CONSTRAINT_PART_H

// Libraries
#include "Test.h"
#include <reactphysics3d/constraint/AxisConstraintPart.h>
#include <vector>
#include <cmath>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestAxisConstraintPart
/**
 * Unit test for the AxisConstraintPart class. The part is pure math on velocities and mass
 * properties, so these tests drive it with hand-made bodies and a minimal integrator (integrate
 * velocities, solve, integrate positions) instead of a PhysicsWorld. That keeps them exact and
 * lets the spring behaviour be checked against the analytical solution.
 */
class TestAxisConstraintPart : public Test {

    private :

        // ---------- Structures ---------- //

        /// A point mass or rigid body as far as the part is concerned
        struct Body {
            Vector3 position;
            Vector3 linearVelocity;
            Vector3 angularVelocity;
            decimal inverseMass;
            Matrix3x3 inverseInertiaTensorWorld;

            Body() : inverseMass(0), inverseInertiaTensorWorld(Matrix3x3::zero()) {}

            /// A dynamic point mass (no rotational inertia)
            static Body pointMass(decimal mass, const Vector3& pos) {
                Body b;
                b.position = pos;
                b.inverseMass = decimal(1.0) / mass;
                return b;
            }

            /// A dynamic body with a diagonal inertia tensor
            static Body rigid(decimal mass, const Vector3& inertia, const Vector3& pos) {
                Body b = pointMass(mass, pos);
                b.inverseInertiaTensorWorld = Matrix3x3(decimal(1.0) / inertia.x, 0, 0,
                                                        0, decimal(1.0) / inertia.y, 0,
                                                        0, 0, decimal(1.0) / inertia.z);
                return b;
            }

            /// A static body
            static Body fixed(const Vector3& pos) {
                Body b;
                b.position = pos;
                return b;
            }

            AxisConstraintBody view() {
                return AxisConstraintBody(&linearVelocity, &angularVelocity, inverseMass, &inverseInertiaTensorWorld);
            }
        };

        /// Result of simulating a spring between two bodies along the Y axis
        struct SpringRun {
            std::vector<decimal> error;      // C at the end of every step
            decimal finalLambda;             // accumulated impulse of the last step
            bool finite;                     // no NaN/inf anywhere
        };

        // ---------- Methods ---------- //

        /// Simulate body2 hanging from body1 on a spring along the +Y axis for nbSteps steps of
        /// timeStep, with an optional constant acceleration on body2 (gravity). Both bodies are
        /// point masses (no lever arms), body1 may be static.
        SpringRun simulateSpring(Body body1, Body body2, decimal restLength, const SpringSettings& spring,
                                 decimal timeStep, int nbSteps, const Vector3& gravity = Vector3(0, 0, 0)) {

            const Vector3 axis(0, 1, 0);
            const Vector3 zero(0, 0, 0);
            AxisConstraintPart part;
            SpringRun run;
            run.finite = true;

            for (int step = 0; step < nbSteps; step++) {

                // Integrate velocities (external forces)
                body2.linearVelocity += gravity * timeStep;

                // Solve
                const decimal C = (body2.position - body1.position).dot(axis) - restLength;
                part.computeSpringConstraintProperties(timeStep, body1.view(), zero, body2.view(), zero, axis, C, spring);
                part.resetTotalLambda();
                part.solveVelocityConstraint(body1.view(), body2.view(), axis, DECIMAL_SMALLEST, DECIMAL_LARGEST);

                // Integrate positions
                body1.position += body1.linearVelocity * timeStep;
                body2.position += body2.linearVelocity * timeStep;

                const decimal newC = (body2.position - body1.position).dot(axis) - restLength;
                if (!std::isfinite(newC)) run.finite = false;
                run.error.push_back(newC);
            }

            run.finalLambda = part.getTotalLambda();
            return run;
        }

        /// Average period of the oscillation of a signal sampled every timeStep, measured from its
        /// zero crossings (linearly interpolated). Returns 0 if there are not enough crossings.
        static decimal measurePeriod(const std::vector<decimal>& signal, decimal timeStep) {
            std::vector<decimal> crossings;
            for (size_t i = 1; i < signal.size(); i++) {
                if ((signal[i-1] < 0) != (signal[i] < 0)) {
                    const decimal t = (decimal(i) - signal[i] / (signal[i] - signal[i-1])) * timeStep;
                    crossings.push_back(t);
                }
            }
            if (crossings.size() < 3) return decimal(0.0);
            // Consecutive same-direction crossings are one period apart
            decimal sum = 0;
            int count = 0;
            for (size_t i = 2; i < crossings.size(); i++) {
                sum += crossings[i] - crossings[i-2];
                count++;
            }
            return sum / decimal(count);
        }

        /// Relative velocity of the two constrained points along the axis (J v)
        static decimal relativeVelocity(const Body& body1, const Vector3& r1, const Body& body2, const Vector3& r2, const Vector3& axis) {
            const Vector3 v1 = body1.linearVelocity + body1.angularVelocity.cross(r1);
            const Vector3 v2 = body2.linearVelocity + body2.angularVelocity.cross(r2);
            return axis.dot(v2 - v1);
        }

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestAxisConstraintPart(const std::string& name) : Test(name) {

        }

        /// Run the tests
        void run() {
            testSpringFrequency();
            testSpringFrequencyUsesEffectiveMass();
            testCriticalDamping();
            testStiffSpringIsStable();
            testGravityEquilibrium();
            testHardConstraintWithLeverArms();
            testImpulseClamping();
            testWarmStart();
            testLockAxisFactors();
            testInactiveWhenBothBodiesStatic();
        }

        /// A 1 Hz spring must oscillate with a 1 s period, measured over several periods
        void testSpringFrequency() {
            const decimal timeStep = decimal(1.0) / decimal(60.0);
            SpringRun run = simulateSpring(Body::fixed(Vector3(0, 0, 0)), Body::pointMass(2.0, Vector3(0, 1.1, 0)), 1.0,
                                           SpringSettings::fromFrequencyAndDampingRatio(1.0, 0.0), timeStep, 300);
            rp3d_test(run.finite);
            const decimal period = measurePeriod(run.error, timeStep);
            rp3d_test(std::abs(period - decimal(1.0)) < decimal(0.03));
            // Implicit integration damps a little; it must never grow
            rp3d_test(std::abs(run.error.back()) < decimal(0.1));
        }

        /// With both bodies dynamic the frequency must still be the configured one: the spring is
        /// defined against the effective (reduced) mass, not against the mass of either body
        void testSpringFrequencyUsesEffectiveMass() {
            const decimal timeStep = decimal(1.0) / decimal(120.0);
            Body body1 = Body::pointMass(1.0, Vector3(0, 0, 0));
            Body body2 = Body::pointMass(3.0, Vector3(0, 1.05, 0));
            SpringRun run = simulateSpring(body1, body2, 1.0, SpringSettings::fromFrequencyAndDampingRatio(2.0, 0.0), timeStep, 480);
            rp3d_test(run.finite);
            const decimal period = measurePeriod(run.error, timeStep);
            rp3d_test(std::abs(period - decimal(0.5)) < decimal(0.015));
        }

        /// A critically damped spring released from rest must return to rest without overshoot
        void testCriticalDamping() {
            const decimal timeStep = decimal(1.0) / decimal(60.0);
            SpringRun run = simulateSpring(Body::fixed(Vector3(0, 0, 0)), Body::pointMass(2.0, Vector3(0, 1.2, 0)), 1.0,
                                           SpringSettings::fromFrequencyAndDampingRatio(1.0, 1.0), timeStep, 180);
            rp3d_test(run.finite);
            bool overshoot = false;
            for (decimal c : run.error) {
                if (c < decimal(-0.001)) overshoot = true;
            }
            rp3d_test(!overshoot);
            rp3d_test(std::abs(run.error.back()) < decimal(0.005));
        }

        /// A spring far beyond the explicit stability limit (k dt^2 / m = 1e8 / 3600) must stay
        /// bounded: this is the whole point of solving it implicitly
        void testStiffSpringIsStable() {
            const decimal timeStep = decimal(1.0) / decimal(60.0);
            SpringRun run = simulateSpring(Body::fixed(Vector3(0, 0, 0)), Body::pointMass(1.0, Vector3(0, 2.0, 0)), 1.0,
                                           SpringSettings::fromStiffnessAndDamping(1.0e8, 0.0), timeStep, 120);
            rp3d_test(run.finite);
            bool bounded = true;
            for (decimal c : run.error) {
                if (std::abs(c) > decimal(1.0)) bounded = false;
            }
            rp3d_test(bounded);
            rp3d_test(std::abs(run.error.back()) < decimal(0.01));
        }

        /// Hanging under gravity a damped spring must settle at x = -m g / k, and the impulse it
        /// applies each step must equal the weight times the time step
        void testGravityEquilibrium() {
            const decimal timeStep = decimal(1.0) / decimal(60.0);
            const decimal mass = 5.0, stiffness = 2000.0, g = 9.81;
            SpringRun run = simulateSpring(Body::fixed(Vector3(0, 0, 0)), Body::pointMass(mass, Vector3(0, 1.0, 0)), 1.0,
                                           SpringSettings::fromStiffnessAndDamping(stiffness, 200.0), timeStep, 300,
                                           Vector3(0, -g, 0));
            rp3d_test(run.finite);
            const decimal expected = -mass * g / stiffness;
            rp3d_test(std::abs(run.error.back() - expected) < std::abs(expected) * decimal(0.01));
            rp3d_test(std::abs(run.finalLambda / timeStep - mass * g) < mass * g * decimal(0.01));
        }

        /// A hard constraint between two rotating bodies with lever arms must zero the relative
        /// velocity of the two points along the axis in one solve and conserve linear momentum
        void testHardConstraintWithLeverArms() {
            const Vector3 axis(0, 1, 0);
            Body body1 = Body::rigid(2.0, Vector3(0.5, 0.5, 0.5), Vector3(0, 0, 0));
            Body body2 = Body::rigid(1.0, Vector3(0.2, 0.3, 0.1), Vector3(0, 1, 0));
            body1.linearVelocity = Vector3(0, 1, 0);
            body1.angularVelocity = Vector3(1, 0, 0.5);
            body2.linearVelocity = Vector3(0, -2, 0);
            body2.angularVelocity = Vector3(0, 0, -1);
            const Vector3 r1(0.3, 0, 0.1);
            const Vector3 r2(-0.2, 0, 0.05);

            const Vector3 momentumBefore = body1.linearVelocity / body1.inverseMass + body2.linearVelocity / body2.inverseMass;

            AxisConstraintPart part;
            part.computeConstraintProperties(body1.view(), r1, body2.view(), r2, axis);
            rp3d_test(part.isActive());
            rp3d_test(!part.isSoft());
            rp3d_test(part.solveVelocityConstraint(body1.view(), body2.view(), axis, DECIMAL_SMALLEST, DECIMAL_LARGEST));

            rp3d_test(std::abs(relativeVelocity(body1, r1, body2, r2, axis)) < decimal(1e-4));
            const Vector3 momentumAfter = body1.linearVelocity / body1.inverseMass + body2.linearVelocity / body2.inverseMass;
            rp3d_test(Vector3::approxEqual(momentumBefore, momentumAfter, decimal(1e-4)));

            // A second iteration has nothing left to do
            rp3d_test(!part.solveVelocityConstraint(body1.view(), body2.view(), axis, DECIMAL_SMALLEST, DECIMAL_LARGEST));
        }

        /// A one-sided constraint (lambda >= 0) stops an approaching body but lets a separating one go
        void testImpulseClamping() {
            const Vector3 axis(0, 1, 0);
            const Vector3 zero(0, 0, 0);
            Body ground = Body::fixed(Vector3(0, 0, 0));

            // Approaching: body 2 moves along -n, towards body 1
            Body approaching = Body::pointMass(1.0, Vector3(0, 1, 0));
            approaching.linearVelocity = Vector3(0, -1, 0);
            AxisConstraintPart part;
            part.computeConstraintProperties(ground.view(), zero, approaching.view(), zero, axis);
            rp3d_test(part.solveVelocityConstraint(ground.view(), approaching.view(), axis, decimal(0.0), DECIMAL_LARGEST));
            rp3d_test(approxEqual(approaching.linearVelocity.y, decimal(0.0), decimal(1e-6)));
            rp3d_test(approxEqual(part.getTotalLambda(), decimal(1.0), decimal(1e-6)));
            rp3d_test(Vector3::approxEqual(ground.linearVelocity, zero));

            // Separating: body 2 moves along +n, the clamp must forbid a pulling impulse
            Body separating = Body::pointMass(1.0, Vector3(0, 1, 0));
            separating.linearVelocity = Vector3(0, 1, 0);
            part.computeConstraintProperties(ground.view(), zero, separating.view(), zero, axis);
            part.resetTotalLambda();
            rp3d_test(!part.solveVelocityConstraint(ground.view(), separating.view(), axis, decimal(0.0), DECIMAL_LARGEST));
            rp3d_test(approxEqual(separating.linearVelocity.y, decimal(1.0), decimal(1e-6)));
            rp3d_test(approxEqual(part.getTotalLambda(), decimal(0.0), decimal(1e-6)));

            // Symmetric clamp: an impulse larger than the budget is cut to the budget
            Body fast = Body::pointMass(2.0, Vector3(0, 1, 0));
            fast.linearVelocity = Vector3(0, -3, 0);
            part.computeConstraintProperties(ground.view(), zero, fast.view(), zero, axis);
            part.resetTotalLambda();
            part.solveVelocityConstraint(ground.view(), fast.view(), axis, decimal(-1.0), decimal(1.0));
            rp3d_test(approxEqual(part.getTotalLambda(), decimal(1.0), decimal(1e-6)));
            rp3d_test(approxEqual(fast.linearVelocity.y, decimal(-2.5), decimal(1e-6)));
        }

        /// Warm starting re-applies the accumulated impulse of the previous step
        void testWarmStart() {
            const Vector3 axis(0, 1, 0);
            const Vector3 zero(0, 0, 0);
            Body ground = Body::fixed(Vector3(0, 0, 0));
            Body body = Body::pointMass(1.0, Vector3(0, 1, 0));
            body.linearVelocity = Vector3(0, -1, 0);

            AxisConstraintPart part;
            part.computeConstraintProperties(ground.view(), zero, body.view(), zero, axis);
            part.solveVelocityConstraint(ground.view(), body.view(), axis, decimal(0.0), DECIMAL_LARGEST);
            const decimal lambda = part.getTotalLambda();
            rp3d_test(lambda > decimal(0.0));

            // Next step: same situation, warm start alone must already solve it
            body.linearVelocity = Vector3(0, -1, 0);
            part.computeConstraintProperties(ground.view(), zero, body.view(), zero, axis);
            part.warmStart(ground.view(), body.view(), axis);
            rp3d_test(approxEqual(body.linearVelocity.y, decimal(0.0), decimal(1e-6)));
            rp3d_test(!part.solveVelocityConstraint(ground.view(), body.view(), axis, decimal(0.0), DECIMAL_LARGEST));
            rp3d_test(approxEqual(part.getTotalLambda(), lambda, decimal(1e-6)));

            part.resetTotalLambda();
            rp3d_test(approxEqual(part.getTotalLambda(), decimal(0.0)));
        }

        /// Lock axis factors must be honoured when applying the impulse
        void testLockAxisFactors() {
            const Vector3 axis(0, 1, 0);
            Body ground = Body::fixed(Vector3(0, 0, 0));
            Body body = Body::rigid(1.0, Vector3(1, 1, 1), Vector3(0, 1, 0));
            body.linearVelocity = Vector3(0, -1, 0);
            const Vector3 r2(0.5, 0, 0);

            // Rotation locked: only the linear velocity may change
            AxisConstraintBody lockedView(&body.linearVelocity, &body.angularVelocity, body.inverseMass,
                                          &body.inverseInertiaTensorWorld, Vector3(1, 1, 1), Vector3(0, 0, 0));
            AxisConstraintPart part;
            part.computeConstraintProperties(ground.view(), Vector3(0, 0, 0), lockedView, r2, axis);
            part.solveVelocityConstraint(ground.view(), lockedView, axis, DECIMAL_SMALLEST, DECIMAL_LARGEST);
            rp3d_test(Vector3::approxEqual(body.angularVelocity, Vector3(0, 0, 0)));
            rp3d_test(body.linearVelocity.y > decimal(-1.0));
        }

        /// Two static bodies cannot be constrained: the part must deactivate itself
        void testInactiveWhenBothBodiesStatic() {
            const Vector3 axis(0, 1, 0);
            const Vector3 zero(0, 0, 0);
            Body a = Body::fixed(Vector3(0, 0, 0));
            Body b = Body::fixed(Vector3(0, 1, 0));
            AxisConstraintPart part;
            part.computeConstraintProperties(a.view(), zero, b.view(), zero, axis);
            rp3d_test(!part.isActive());
            part.computeSpringConstraintProperties(decimal(1.0) / decimal(60.0), a.view(), zero, b.view(), zero, axis,
                                                   decimal(0.5), SpringSettings::fromFrequencyAndDampingRatio(1.0, 0.5));
            rp3d_test(!part.isActive());
            rp3d_test(!part.solveVelocityConstraint(a.view(), b.view(), axis, DECIMAL_SMALLEST, DECIMAL_LARGEST));
        }
};

}

#endif
