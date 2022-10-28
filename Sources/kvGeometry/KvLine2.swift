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
//  KvLine2.swift
//  kvKit
//
//  Created by Svyatoslav Popov on 21.09.2022.
//

import kvKit



/// Implementation of a line in 2D coordinate space.
///
/// Lines have orientation so they have directions and are equal to left half-spaces relative to the directions.
public struct KvLine2<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector2
    public typealias Coordinate = Vector

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>



    /// A vector orthogonal to the receiver.
    public var normal: Vector
    /// The offset between the receiver and the origin.
    ///
    /// - Note: Canonical line equateion: *normal*∙*x* + *c* = 0, where *x* is a coordinate on line.
    /// - Note: If the *normal* is a unit vector then *c* is the negated distance from the receiver to the origin.
    public var c: Scalar



    /// Memberwise initializer.
    @inlinable
    public init(normal: Vector, c: Scalar) {
        self.normal = normal
        self.c = c
    }


    /// A line where *a*∙*x* + *b*∙*y* + *c* = 0, where (*x*, *y*) is a coordinate on the line.
    @inlinable public init(a: Scalar, b: Scalar, c: Scalar) { self.init(normal: Vector(x: a, y: b), c: c) }


    /// A line where *a*∙*x* + *b*∙*y* + *c* = 0, where (*x*, *y*) is a coordinate on the line, (*a*, *b*, *c*) are equal to corresponding elements of *abc* vector.
    @inlinable public init(abc: Math.Vector3) { self.init(a: abc.x, b: abc.y, c: abc.z) }


    /// A line containing both *c0* and *c1* coordinates.
    ///
    /// - Note: Resulting line is degenerate when *c0* is equal to *c1*.
    @inlinable public init(_ c0: Coordinate, _ c1: Coordinate) { self.init(in: c1 - c0, at: c0) }


    /// A line containing both *v0* and *v1* vertices.
    ///
    /// - Note: Resulting line is degenerate when coordinates of *v0* and *v1* are equal.
    @inlinable
    public init<V0, V1>(_ v0: V0, _ v1: V1)
    where V0 : KvVertex2Protocol, V0.Math == Math, V1 : KvVertex2Protocol, V1.Math == Math {
        self.init(in: v1.coordinate - v0.coordinate, at: v0)
    }


    @inlinable
    public init<V>(_ ray: KvRay2<V>)
    where V : KvVertex2Protocol, V.Math == Math {
        self.init(in: ray.front, at: ray.origin.coordinate)
    }


    /// A line having given direction and containing given coordinate.
    @inlinable
    public init(in direction: Vector, at coordinate: Coordinate) {
        self.init(normal: Vector(x: -direction.y, y: direction.x), at: coordinate)
    }


    /// A line having given direction and containing given coordinate.
    @inlinable
    public init<V>(in direction: Vector, at vertex: V)
    where V : KvVertex2Protocol, V.Math == Math {
        self.init(normal: Vector(x: -direction.y, y: direction.x), at: vertex.coordinate)
    }


    /// A line having given direction and distance to coordinate origin.
    @inlinable
    public init(in direction: Vector, c: Scalar) {
        self.init(normal: Vector(x: -direction.y, y: direction.x), c: c)
    }


    /// A line having given normal and containing given coordinate.
    @inlinable
    public init(normal: Vector, at coordinate: Coordinate) {
        self.init(normal: normal, c: -Math.dot(coordinate, normal))
    }


    /// A line having given normal and containing given coordinate.
    @inlinable
    public init<V>(normal: Vector, at vertex: V)
    where V : KvVertex2Protocol, V.Math == Math {
        self.init(normal: normal, c: -Math.dot(vertex.coordinate, normal))
    }



    // MARK: Operations

    /// A coordinate on the receiver having minimum distance to the coordinate origin.
    @inlinable public var closestToOrigin: Coordinate { normal * (-c / Math.length²(normal)) }
    /// Some coordinate on the receiver.
    @inlinable public var anyCoordinate: Coordinate { closestToOrigin }

    /// - Note: The direction is the receiver's normal rotated by –90°.
    @inlinable public var front: Vector { Vector(x: normal.y, y: -normal.x) }

    @inlinable public var isDegenerate: Bool { Math.isZero(normal) }


    /// - Returns: Value of the canonical equation at *x*: *normal* · *x* + *c*.
    ///
    /// - Note: Returned value is the signed offset. It's positive when *x* is in the halfspace the normal is aimed at.
    /// - Note: Signed offset matches signed distance when the normal is a unit vector.
    @inlinable public func at(_ x: Coordinate) -> Scalar { Math.dot(normal, x) + c }


    /// - Returns: A numeric tolerance for result of ``at(_:)`` with given coordinate.
    @inlinable public func epsArg(at x: Coordinate) -> Math.EpsArg { Math.epsArg(normal).dot(Math.epsArg(x)) + Math.EpsArg(c) }


    /// - Returns: Y coodinate where vertical line at *x* intersects the receiver.
    @inlinable
    public func y(x: Scalar) -> Scalar? {
        guard KvIsNonzero(normal.y) else { return nil }

        return -(c + normal.x * x) / normal.y
    }

    /// - Returns: X coodinate where horizontal line at *y* intersects the receiver.
    @inlinable
    public func x(y: Scalar) -> Scalar? {
        guard KvIsNonzero(normal.x) else { return nil }

        return -(c + normal.y * y) / normal.x
    }


    /// Alias to ``at``(method).
    @inlinable
    public func signedOffset(to x: Coordinate) -> Scalar { at(x) }


    /// - Returns: The distance from the receiver to *c* divided by length of the normal.
    @inlinable
    public func offset(to x: Coordinate) -> Scalar { abs(at(x)) }


    /// - Returns: A boolean value indicating whether the receiver contains given coordinate.
    ///
    /// - SeeAlso: ``isInPositive``, ``isInNegative``, ``location(of:)``
    @inlinable public func contains(_ x: Coordinate) -> Bool { KvIsZero(at(x), eps: epsArg(at: x).tolerance) }

    /// - Returns: A boolean value indicating whether the receiver contains coordinates of given ray.
    @inlinable public func contains<V>(_ ray: KvRay2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Math
    {
        contains(ray.origin.coordinate)
        && KvIsZero(Math.dot(normal, ray.front), eps: Math.epsArg(normal).dot(Math.epsArg(ray.front)).tolerance)
    }


    /// - Returns: A boolean value indicating whether given coordinate is in the positive half-space.
    ///
    /// - SeeAlso: ``isInNegative``, ``contains(_:)``, ``location(of:)``
    /// - Note: A half-space is positive or negative whether the normal vector is in it.
    @inlinable public func isInPositive(_ x: Coordinate) -> Bool { KvIsPositive(at(x), eps: epsArg(at: x).tolerance) }


    /// - Returns: A boolean value indicating whether given coordinate is in the negative half-space.
    ///
    /// - SeeAlso: ``isInPositive``, ``contains(_:)``, ``location(of:)``
    /// - Note: A half-space is positive or negative whether the normal vector is in it.
    @inlinable public func isInNegative(_ x: Coordinate) -> Bool { KvIsNegative(at(x), eps: epsArg(at: x).tolerance) }


    /// - Returns: A location of given coordinate relative to the receiver.
    ///
    /// - Note: It's effective when at least two cases should be hanled. It only one case is handled then consider ``isInPositive``, ``isInNegative`` or ``contains(_:)``.
    @inlinable
    public func location(of x: Coordinate) -> Location {
        var isNegative = false

        if KvIsPositive(at(x), eps: epsArg(at: x).tolerance, alsoIsNegative: &isNegative) {
            return .positive
        }
        else if isNegative {
            return .negative
        }
        else { return .neutral }
    }


    /// - Returns: A boolean value indicating whether the receiver and *rhs* intersect.
    @inlinable public func intersects(with rhs: Self) -> Bool { KvIsNonzero(normal.x * rhs.normal.y - normal.y * rhs.normal.x) }


    /// - Returns: Single common coordinate of the receiver and given line. If the lines are parallel or equal then *nil* is returned.
    @inlinable
    public func intersection(with line: Self) -> Coordinate? {
        let denominator = normal.x * line.normal.y - normal.y * line.normal.x

        guard KvIsNonzero(denominator) else { return nil }

        return Coordinate(x: line.c * normal.y - c * line.normal.y, y: c * line.normal.x - line.c * normal.x) / denominator
    }


    /// - Returns: The closest coordinate on the receiver to given coordinate.
    @inlinable
    public func projection(of x: Coordinate) -> Coordinate { x - at(x) * normal }


    /// Normalizes the receiver's normal.
    @inlinable
    public mutating func normalize() {
        let scale = Math.rsqrt(Math.length²(normal))

        normal *= scale
        c *= scale
    }


    /// Inverses the receiver's orientation.
    @inlinable public mutating func negate() { self = -self }


    /// Normalizes the receiver's direction if it isn't a zero vector.
    @inlinable
    public func normalized() -> Self {
        let scale = Math.rsqrt(Math.length²(normal))

        return Self(normal: normal * scale, c: c * scale)
    }


    /// - Returns: A line matching the receiver but having unit normal.
    @inlinable
    public mutating func safeNormalize() {
        guard Math.isNonzero(normal) else { return }

        normalize()
    }


    /// - Returns: A line matching the receiver but having unit normal when the receiver's normal isn't a zero vector.
    @inlinable
    public func safeNormalized() -> Self? {
        guard Math.isNonzero(normal) else { return nil }

        return normalized()
    }


    /// Translates all the receiver's ponts by *offset*.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func translate(by offset: Vector) {
        c -= Math.dot(normal, offset)
    }


    /// - Returns: A line produced applying translation by *offset* to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func translated(by offset: Vector) -> Self {
        Self(normal: normal, c: c - Math.dot(normal, offset))
    }


    /// Scales all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func scale(by scale: Scalar) {
        c *= scale
    }


    /// - Returns: A line produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        Self(normal: normal, c: c * scale)
    }


    /// Rotates all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func rotate(by angle: Scalar) {
        normal = AffineTransform(angle: angle).act(normal: normal)
    }


    /// - Returns: A line produced applying rotation to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func rotated(by angle: Scalar) -> Self {
        Self(normal: AffineTransform(angle: angle).act(normal: normal), c: c)
    }


    /// Applies given transformation to all the receiver's points.
    @inlinable public mutating func apply(_ t: Transform) { self = t * self }

    /// Applies given transformation to all the receiver's points.
    @inlinable public mutating func apply(_ t: AffineTransform) { self = t * self }



    // MARK: Operators

    /// - Returns: A line mathing given line but having opposite normal.
    @inlinable
    public static prefix func -(rhs: Self) -> Self {
        Self(normal: -rhs.normal, c: -rhs.c)
    }


    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        Self(normal: lhs.act(normal: rhs.normal), at: lhs.act(coordinate: rhs.anyCoordinate))
    }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        Self(normal: lhs.act(normal: rhs.normal), at: lhs.act(coordinate: rhs.anyCoordinate))
    }



    // MARK: .Location

    public enum Location {

        /// Denotes part of the coordinate space where value of the line canonical equation is numerically positive.
        case positive
        /// Denotes part of the coordinate space where value of the line canonical equation is numerically negative.
        case negative
        /// Denotes part of the coordinate space where value of the line canonical equation is numerically zero. In other words coordinates of the line.
        case neutral

    }

}



// MARK: : KvNumericallyEquatable

extension KvLine2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        switch KvIsNonzero(c) {
        case true:
            guard KvIsNonzero(rhs.c) else { return false }

            return Math.isEqual(normal * rhs.c, rhs.normal * c)

        case false:
            return KvIsZero(rhs.c) && Math.isCoDirectional(normal, rhs.normal)
        }
    }

}



// MARK: : Equatable

extension KvLine2 : Equatable { }



// MARK: : Hashable

extension KvLine2 : Hashable { }



// MARK: : ExpressibleByArrayLiteral

extension KvLine2 : ExpressibleByArrayLiteral {

    /// Initializes the receiver from array if the canonical line parameters: a, b, c.
    @inlinable
    public init(arrayLiteral abc: Math.Scalar...) {
        KvDebug.assert(abc.count == 3, "Unexpected number (\(abc.count)) of line parameters")

        self.init(normal: Vector(abc[0], abc[1]), c: abc[2])
    }

}
