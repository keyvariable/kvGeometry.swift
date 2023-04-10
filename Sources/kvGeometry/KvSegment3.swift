//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2022 Svyatoslav Popov (info@keyvar.com).
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
//  KvSegment3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 27.10.2022.
//

import kvKit



/// Implementation of a segment in 3D coordinate space.
///
/// The segment coordinates are represented with *endPoints.0* + *direction* · *step* equation where *step* ∈ [0, *length*]. See ``at(_:)`` method.
public struct KvSegment3<Vertex : KvVertex3Protocol> {

    public typealias Math = Vertex.Math
    public typealias Vertex = Vertex

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector3

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>



    /// End points of the segment.
    public let endPoints: (Vertex, Vertex)

    /// Direction unit vector from *endPoints.0* to *endPoints.1*.
    public let front: Vector
    /// Length of the receiver.
    public let length: Scalar
    /// Reciprocal of the receiver's length.
    public let length⁻¹: Scalar



    /// Memberwise inilializer.
    @usableFromInline
    internal init(endPoints: (Vertex, Vertex), direction: Vector, length: Scalar, length⁻¹: Scalar) {
        self.endPoints = endPoints
        self.front = direction
        self.length = length
        self.length⁻¹ = length⁻¹
    }


    /// Initalizes a segment with given end ponts.
    ///
    /// - Note: The caller is responsible for clonning provided end points.
    @inlinable
    public init(_ endPoints: (Vertex, Vertex)) {
        switch Math.isInequal(endPoints.0.coordinate, endPoints.1.coordinate) {
        case true:
            let vector = endPoints.1.coordinate - endPoints.0.coordinate
            let length = Math.length(vector)
            let length⁻¹ = 1 / length

            self.init(endPoints: endPoints, direction: vector * length⁻¹, length: length, length⁻¹: length⁻¹)

        case false:
            self.init(endPoints: endPoints, direction: .unitX, length: 0, length⁻¹: 0)
        }
    }

    /// Initalizes a segment with given end ponts.
    ///
    /// - Note: The caller is responsible for clonning provided end points.
    @inlinable public init(_ origin: Vertex, _ target: Vertex) { self.init((origin, target)) }



    // MARK: Operations

    @inlinable
    public var isDegenerate: Bool { KvIsZero(length) }


    /// - Returns: Linear combination of the receiver's endPoints where `at(0) == endPoints.0` and `at(length) == endPoints.1`.
    ///
    /// See ``coordinate(at:)``.
    @inlinable public func at(_ step: Scalar) -> Vertex { endPoints.0.mixed(endPoints.1, t: step * length⁻¹) }


    /// - Returns: The same coordinate as `at(step).coordinate`.
    ///
    /// See ``at(_:)``.
    @inlinable public func coordinate(at step: Scalar) -> Vertex.Coordinate { endPoints.0.coordinate + step * front }


    /// - Returns: Copy of the receiver where vertices are cloned.
    @inlinable public func clone() -> Self { Self(endPoints: (endPoints.0.clone(), endPoints.1.clone()), direction: front, length: length, length⁻¹: length⁻¹) }


    /// - Returns: A ray having the same direction but flipped origin.
    @inlinable public func flipped() -> Self { Self(endPoints: (endPoints.0.flipped(), endPoints.1.flipped()), direction: front, length: length, length⁻¹: length⁻¹) }


    /// - Returns: Step *t* where `at(t)` is a coordinate the receiver intersects given plane.
    ///
    /// - Note: It's equal to distance from *endPoints.0* to the intersection coordinate.
    ///
    /// See ``at(_:)``, ``intersection(with:)``, ``intersects(with:)``.
    @inlinable
    public func step(to plane: KvPlane3<Math>) -> Scalar? {
        let divider = Math.dot(plane.normal, front)

        guard KvIsNonzero(divider) else { return nil }

        let t = -plane.at(endPoints.0.coordinate) / divider

        guard contains(step: t) else { return nil }

        return t
    }


    /// - Returns: Distance to a coordinate.
    @inlinable
    public func distance(to c: Vertex.Coordinate) -> Scalar {
        let arg = (vector: c - endPoints.0.coordinate, epsArg: Math.epsArg(c) - Math.epsArg(endPoints.0.coordinate))
        let cross = Math.cross(front, arg.vector)

        let isOnLine = Math.isZero(cross, eps: Math.epsArg(front).cross(arg.epsArg).tolerance)
        /// Step to the projection of the coordinate.
        let step = Math.dot(front, arg.vector)

        guard KvIsNotNegative(step)
        else { return isOnLine ? -step : Math.length(arg.vector) }

        guard KvIs(step, lessThanOrEqualTo: length)
        else { return isOnLine ? step - length : Math.length(c - endPoints.1.coordinate) }

        return isOnLine ? 0 : Math.length(cross)
    }

    /// - Returns: Distance to a vertex.
    @inlinable public func distance(to v: Vertex) -> Scalar { distance(to: v.coordinate) }

    /// - Returns: Distance to a vertex.
    @inlinable
    public func distance<V>(to v: V) -> Scalar
    where V : KvVertex3Protocol, V.Math == Math {
        distance(to: v.coordinate)
    }

    /// - Returns: Distance to a segment.
    @inlinable
    public func distance(to s: Self) -> Scalar {
        let (lFront, lOrigin) = (front, endPoints.0.coordinate)
        let (rFront, rOrigin) = (s.front, s.endPoints.0.coordinate)

        let n = Math.cross(lFront, rFront)
        let n2 = Math.cross(rFront, n)

        let denominator = Math.dot(lFront, n2)

        switch KvIsNonzero(denominator) {
        case true:
            let cross = Math.cross(n, rOrigin - lOrigin)
            let denominator⁻¹ = 1 / denominator

            let lt = Math.dot(rFront, cross) * denominator⁻¹
            let rt = Math.dot(lFront, cross) * denominator⁻¹

            return Math.distance(coordinate(at: clamp(lt, 0, length)),
                                 s.coordinate(at: clamp(rt, 0, s.length)))

        case false:
            let v00 = s.endPoints.0.coordinate - endPoints.0.coordinate
            let v01 = s.endPoints.1.coordinate - endPoints.0.coordinate

            // Projections of s.endPoints.0 and s.endPoints.1 on the receiver.
            let (p0, p1): (Scalar, Scalar) =
            { KvIs($0, lessThanOrEqualTo: $1) ? ($0, $1) : ($1, $0) }(Math.dot(v00, front), Math.dot(v01, front))

            guard KvIsPositive(p1) else { return Math.distance(endPoints.0.coordinate, s.endPoints.1.coordinate) }
            guard KvIs(p0, lessThan: length) else { return Math.distance(endPoints.1.coordinate, s.endPoints.0.coordinate) }

            // It's true due to direction is a unit vector.
            return Math.length(Math.cross(v00, front))
        }
    }


    /// - Returns: A boolean value indicating whether the receiver contains `at(step)` coordinate.
    ///
    /// See ``at(_:).
    @inlinable public func contains(step: Scalar) -> Bool { KvIsNotNegative(step) && KvIs(step, lessThanOrEqualTo: length) }


    /// - Returns: A boolean value indicating whether the receiver contains given coordinate.
    @inlinable
    public func contains(_ c: Vertex.Coordinate) -> Bool {
        let arg = (vector: c - endPoints.0.coordinate, epsArg: Math.epsArg(c) - Math.epsArg(endPoints.0.coordinate))

        guard Math.isZero(Math.cross(front, arg.vector), eps: Math.epsArg(front).cross(arg.epsArg).tolerance) else { return false }

        return contains(step: Math.dot(front, arg.vector))
    }

    /// - Returns: A boolean value indicating whether the receiver contains given vertex.
    @inlinable public func contains(_ v: Vertex) -> Bool { contains(v.coordinate) }


    /// - Returns: A boolean value indicating whether the receiver intersects given plane.
    ///
    /// See ``step(to:)``, ``intersection(with:)``.
    @inlinable public func intersects(with plane: KvPlane3<Math>) -> Bool { step(to: plane) != nil }


    /// - Returns: A copy of the origin translated to coordinate where the receiver and given plane intersect.
    ///
    /// See ``step(to:)``, ``intersects(with:)``.
    @inlinable
    public func intersection(with plane: KvPlane3<Math>) -> Vertex? {
        step(to: plane).map(self.at(_:))
    }


    /// - Returns: A segment produced applying translation by *offset* to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func translated(by offset: Vector) -> Self {
        Self(endPoints: (endPoints.0 + offset, endPoints.1 + offset), direction: front, length: length, length⁻¹: length⁻¹)
    }


    /// - Returns: A ray produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply arbitraty transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        let t = AffineTransform(scale: scale)
        let endPoints = (t * endPoints.0, t * endPoints.1)

        let scale = abs(scale)
        /// Assuming scale >= 0
        switch KvIsPositive(scale) {
        case true:
            return Self(endPoints: endPoints, direction: front, length: length * scale, length⁻¹: length⁻¹ / scale)
        case false:
            return Self(endPoints: endPoints, direction: .unitX, length: 0, length⁻¹: 0)
        }
    }


    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable public static func *(lhs: Transform, rhs: Self) -> Self { Self(lhs * rhs.endPoints.0, lhs * rhs.endPoints.1) }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable public static func *(lhs: AffineTransform, rhs: Self) -> Self { Self(lhs * rhs.endPoints.0, lhs * rhs.endPoints.1) }

}



// MARK: : KvNumericallyEquatable

extension KvSegment3 : KvNumericallyEquatable where Vertex : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(endPoints.0.coordinate, rhs.endPoints.0.coordinate)
        && Math.isCoDirectional(endPoints.1.coordinate, rhs.endPoints.1.coordinate)
    }

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual<V>(to rhs: KvSegment3<V>) -> Bool
    where V : KvVertex3Protocol, V.Math == Vertex.Math
    {
        Math.isEqual(endPoints.0.coordinate, rhs.endPoints.0.coordinate)
        && Math.isCoDirectional(endPoints.1.coordinate, rhs.endPoints.1.coordinate)
    }

}



// MARK: : Equatable

extension KvSegment3 : Equatable where Vertex : Equatable {

    @inlinable static public func ==(lhs: Self, rhs: Self) -> Bool { lhs.endPoints == rhs.endPoints }

}



// MARK: : Hashable

extension KvSegment3 : Hashable where Vertex : Hashable {

    @inlinable
    public func hash(into hasher: inout Hasher) {
        hasher.combine(endPoints.0)
        hasher.combine(endPoints.1)
    }

}
