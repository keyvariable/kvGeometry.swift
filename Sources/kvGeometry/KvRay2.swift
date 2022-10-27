//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2021 Svyatoslav Popov.
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
//  KvRay2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 19.09.2022.
//

import kvKit



/// Implementation of a ray in 2D coordinate space.
public struct KvRay2<Vertex : KvVertex2Protocol> {

    public typealias Math = Vertex.Math
    public typealias Vertex = Vertex

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector2

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>



    /// A vertex the ray starts at.
    public var origin: Vertex
    /// Direction vector of the ray.
    public var direction: Vector



    /// Memberwise initializer.
    ///
    /// - Note: The caller is responsible for clonning provided origin.
    @inlinable
    public init(in direction: Vector, at origin: Vertex) {
        KvDebug.assert(Math.isNonzero(direction), "Invalid argument: direction of a ray is a zero vector")

        self.origin = origin
        self.direction = direction
    }


    /// - Note: Target is not an instance of *Vertex* to preserve unambiguity in some cases.
    /// - Note: The caller is responsible for clonning provided origin.
    @inlinable
    public init(from origin: Vertex, to target: Vertex.Coordinate) {
        self.init(in: target - origin.coordinate, at: origin)
    }



    // MARK: Operations

    @inlinable
    public var isDegenerate: Bool { Math.isZero(direction) }


    /// - Returns: Value of the canonical equation at *t*: *origin* + *direction* · *t*.
    @inlinable
    public func at(_ t: Scalar) -> Vertex { origin + direction * t }


    /// Inverses the direction preserving the origin.
    @inlinable public mutating func negate() { direction = -direction }


    /// - Returns: Copy of the receiver where vertices are cloned.
    @inlinable public func clone() -> Self { Self(in: direction, at: origin.clone()) }


    /// Flips the origin preserving the direction.
    @inlinable public mutating func flip() { origin.flip() }


    /// - Returns: A ray having the same direction but flipped origin.
    @inlinable public func flipped() -> Self { Self(in: direction, at: origin.flipped()) }


    /// Normalizes the receiver's direction.
    @inlinable
    public mutating func normalize() {
        direction = Math.normalize(direction)
    }


    /// Normalizes the receiver's direction if it isn't a zero vector.
    @inlinable
    public func normalized() -> Self {
        .init(in: Math.normalize(direction), at: origin.clone())
    }


    /// - Returns: A ray matching the receiver but having unit direction.
    @inlinable
    public mutating func safeNormalize() {
        guard let unitDirection = Math.safeNormalize(direction) else { return }

        direction = unitDirection
    }


    /// - Returns: A ray matching the receiver but having unit direction when the receiver's direction isn't a zero vector.
    @inlinable
    public func safeNormalized() -> Self? {
        Math.safeNormalize(direction).map { Self.init(in: $0, at: origin.clone()) }
    }


    /// - Returns: *T* where *origin* + *direction* · *t* is a coordinate the receiver intersects given ray.
    ///
    /// - Note: It's equal to distance to the intersection coordinate when the receiver has unit direction.
    @inlinable
    public func offset<V>(to ray: KvRay2<V>) -> Scalar?
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        let denominator = direction.y * ray.direction.x - direction.x * ray.direction.y

        guard KvIsNonzero(denominator) else { return nil }

        let recip_d = 1 / denominator
        let dOrigin = ray.origin.coordinate - origin.coordinate

        let t = (dOrigin.y * ray.direction.x - dOrigin.x * ray.direction.y) * recip_d
        let tr = (dOrigin.y * direction.x - dOrigin.x * direction.y) * recip_d

        guard KvIsNotNegative(t), KvIsNotNegative(tr) else { return nil }

        return t
    }

    /// - Returns: The argument of the canonical equation the receiver and given line intersect at.
    ///
    /// - Note: It's equal to distance to the intersection coordinate when the receiver has unit direction.
    @inlinable
    public func offset(to line: KvLine2<Math>) -> Scalar? {
        let divider = Math.dot(line.normal, direction)

        guard KvIsNonzero(divider) else { return nil }

        let t = -line.at(origin.coordinate) / divider

        guard KvIsNotNegative(t) else { return nil }

        return t
    }


    /// - Returns: A boolean value indicating whether the receiver intersects given ray.
    @inlinable public func intersects<V>(with ray: KvRay2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        offset(to: ray) != nil
    }

    /// - Returns: A boolean value indicating whether the receiver intersects given line.
    @inlinable public func intersects(with line: KvLine2<Math>) -> Bool { offset(to: line) != nil }


    /// - Returns: A copy of the origin translated to coordinate where the receiver and given ray intersect.
    @inlinable
    public func intersection<V>(with ray: KvRay2<V>) -> Vertex?
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        offset(to: ray).map(self.at(_:))
    }

    /// - Returns: A copy of the origin translated to coordinate where the receiver and given line intersect.
    @inlinable
    public func intersection(with line: KvLine2<Math>) -> Vertex? {
        offset(to: line).map(self.at(_:))
    }


    /// - Returns: Y coodinate where vertical line at *x* intersects the receiver.
    @inlinable
    public func y(x: Scalar) -> Scalar? {
        guard KvIsNonzero(direction.x) else { return nil }

        let t = (x - origin.coordinate.x) / direction.x

        guard KvIsNotNegative(t) else { return nil }

        return origin.coordinate.y + direction.y * t
    }


    /// - Returns: X coordinate for y coordinate or nil whether the receiver is not horizontal.
    @inlinable
    public func x(y: Scalar) -> Scalar? {
        guard KvIsNonzero(direction.y) else { return nil }

        let t = (y - origin.coordinate.y) / direction.y

        guard KvIsNotNegative(t) else { return nil }

        return origin.coordinate.x + direction.x * t
    }


    /// Translates all the receiver's ponts by *offset*.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func translate(by offset: Vector) {
        origin += offset
    }


    /// - Returns: A ray produced applying translation by *offset* to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func translated(by offset: Vector) -> Self {
        Self(in: direction, at: origin + offset)
    }


    /// Scales all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func scale(by scale: Scalar) {
        direction *= scale
        origin.apply(AffineTransform(scale: scale))
    }


    /// - Returns: A ray produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        Self(in: direction * scale, at: AffineTransform(scale: scale) * origin)
    }


    /// Applies given transformation to all the receiver's points.
    @inlinable public mutating func apply(_ t: Transform) { self = t * self }

    /// Applies given transformation to all the receiver's points.
    @inlinable public mutating func apply(_ t: AffineTransform) { self = t * self }



    // MARK: Operators

    /// - Returns: A ray with the same origin but opposite direction.
    @inlinable public static prefix func -(rhs: Self) -> Self { Self(in: -rhs.direction, at: rhs.origin.clone()) }


    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        .init(in: lhs.act(vector: rhs.direction), at: lhs * rhs.origin)
    }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        .init(in: lhs.act(vector: rhs.direction), at: lhs * rhs.origin)
    }

}



// MARK: : KvNumericallyEquatable

extension KvRay2 : KvNumericallyEquatable where Vertex : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(origin.coordinate, rhs.origin.coordinate) && Math.isCoDirectional(direction, rhs.direction)
    }

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual<V>(to rhs: KvRay2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        Math.isEqual(origin.coordinate, rhs.origin.coordinate) && Math.isCoDirectional(direction, rhs.direction)
    }

}



// MARK: : Equatable

extension KvRay2 : Equatable where Vertex : Equatable { }



// MARK: : Hashable

extension KvRay2 : Hashable where Vertex : Hashable { }
