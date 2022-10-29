//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2022 Svyatoslav Popov.
//
//  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
//  the License. You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
//  an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
//  specific language governing permissions and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0
//
//===----------------------------------------------------------------------===//
//
//  KvOriginRay2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.10.2022.
//

import kvKit



/// Implementation of a ray from the coordinate origin in 2D coordinate space. It's faster than ``KvRay2``.
public struct KvOriginRay2<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector2
    public typealias Coordinate = Math.Vector2

    public typealias AffineTransform = KvAffineTransform2<Math>


    /// Direction vector of the receiver.
    public var front: Vector



    /// Memberwise initializer.
    ///
    /// - Note: The caller is responsible for clonning provided origin.
    @inlinable
    public init(in front: Vector) {
        KvDebug.assert(Math.isNonzero(front), "Invalid argument: front of an origin ray is a zero vector")

        self.front = front
    }



    // MARK: Operations

    /// A boolean valie indicating whether the receiver's direction is a zero vector.
    @inlinable public var isDegenerate: Bool { Math.isZero(front) }


    /// - Returns: *front* · *step*.
    @inlinable public func at(_ step: Scalar) -> Coordinate { front * step }


    /// Inverses the direction.
    @inlinable public mutating func negate() { front = -front }


    /// Normalizes the receiver's direction.
    @inlinable
    public mutating func normalize() {
        front = Math.normalize(front)
    }


    /// Normalizes the receiver's direction if it isn't a zero vector.
    @inlinable
    public func normalized() -> Self {
        Self(in: Math.normalize(front))
    }


    /// - Returns: A ray matching the receiver but having unit direction.
    @inlinable
    public mutating func safeNormalize() {
        guard let unitFront = Math.safeNormalize(front) else { return }

        front = unitFront
    }


    /// - Returns: A ray matching the receiver but having unit direction when the receiver's direction isn't a zero vector.
    @inlinable
    public func safeNormalized() -> Self? {
        Math.safeNormalize(front).map { Self.init(in: $0) }
    }


    /// - Returns: Step *t* where `at(t)` is a coordinate the receiver intersects given ray.
    ///
    /// - Note: It's equal to distance from coordinate origin to the intersection coordinate when the receiver has unit direction.
    ///
    /// See ``at(_:)``.
    @inlinable
    public func step<V>(to ray: KvRay2<V>) -> Scalar?
    where V : KvVertex2Protocol, V.Math == Math {
        let denominator = front.y * ray.front.x - front.x * ray.front.y

        guard KvIsNonzero(denominator) else { return nil }

        let recip_d = 1 / denominator
        let dOrigin = ray.origin.coordinate

        let t = (dOrigin.y * ray.front.x - dOrigin.x * ray.front.y) * recip_d
        let tr = (dOrigin.y * front.x - dOrigin.x * front.y) * recip_d

        guard KvIsNotNegative(t), KvIsNotNegative(tr) else { return nil }

        return t
    }

    /// - Returns: Step *t* where `at(t)` is a coordinate the receiver intersects given line.
    ///
    /// - Note: It's equal to distance from coordinate origin to the intersection coordinate when the receiver has unit direction.
    ///
    /// See ``at(_:)``.
    @inlinable
    public func step(to line: KvLine2<Math>) -> Scalar? {
        let divider = Math.dot(line.normal, front)

        guard KvIsNonzero(divider) else { return nil }

        let t = -line.c / divider

        guard KvIsNotNegative(t) else { return nil }

        return t
    }


    /// - Returns: A boolean value indicating whether the receiver intersects given ray.
    @inlinable public func intersects<V>(with ray: KvRay2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Math {
        step(to: ray) != nil
    }

    /// - Returns: A boolean value indicating whether the receiver intersects given plane.
    @inlinable public func intersects(with line: KvLine2<Math>) -> Bool { step(to: line) != nil }


    /// - Returns: A copy of the origin translated to coordinate where the receiver and given ray intersect.
    @inlinable
    public func intersection<V>(with ray: KvRay2<V>) -> Coordinate?
    where V : KvVertex2Protocol, V.Math == Math {
        step(to: ray).map(self.at(_:))
    }

    /// - Returns: A copy of the origin translated to coordinate where the receiver and given plane intersect.
    @inlinable
    public func intersection(with line: KvLine2<Math>) -> Coordinate? {
        step(to: line).map(self.at(_:))
    }


    /// Scales all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func scale(by scale: Scalar) {
        front *= scale
    }


    /// - Returns: A ray produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        Self(in: front * scale)
    }


    /// Applies given transformation to all the receiver's points.
    @inlinable public mutating func apply(_ t: AffineTransform) { self = t * self }



    // MARK: Operators

    /// - Returns: A ray with the same origin but opposite direction.
    @inlinable public static prefix func -(rhs: Self) -> Self { Self(in: -rhs.front) }


    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        Self(in: lhs.act(vector: rhs.front))
    }

}



// MARK: : KvNumericallyEquatable

extension KvOriginRay2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isCoDirectional(front, rhs.front)
    }


    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual<V>(to rhs: KvRay2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Math
    {
        Math.isZero(rhs.origin.coordinate) && Math.isCoDirectional(front, rhs.front)
    }

}



// MARK: : Equatable

extension KvOriginRay2 : Equatable { }



// MARK: : Hashable

extension KvOriginRay2 : Hashable { }
