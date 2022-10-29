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
//  KvLine3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 24.09.2022.
//

import kvKit



/// Implementation of line in 3D coordinate space.
public struct KvLine3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector3
    public typealias Coordinate = Vector

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>



    /// Direction vector of the line having unit length.
    public let front: Vector
    /// The coordinate on the line having minimum distance to the coordinate origin.
    public let origin: Coordinate



    /// Memberwise initializer.
    @usableFromInline
    internal init(front: Vector, origin: Coordinate) {
        self.front = front
        self.origin = origin
    }


    /// A line having given direction and containing given coordinate.
    @inlinable
    public init(in direction: Vector, at coordinate: Coordinate) {
        self.front = Math.normalize(direction)
        self.origin = coordinate - Math.dot(coordinate, self.front)
    }


    /// A line having given direction and containing given vertex.
    @inlinable
    public init<V>(in direction: Vector, at vertex: V)
    where V : KvVertex3Protocol, V.Math == Math {
        self.init(in: direction, at: vertex.coordinate)
    }


    /// A line containing two given coordinates.
    ///
    /// - Note: Initialized line is degenerate if given coordinates are equal.
    @inlinable public init(_ c0: Coordinate, _ c1: Coordinate) { self.init(in: c1 - c0, at: c0) }


    /// A line containing both *v0* and *v1* vertices.
    ///
    /// - Note: Resulting line is degenerate when coordinates of *v0* and *v1* are equal.
    @inlinable
    public init<V0, V1>(_ v0: V0, _ v1: V1)
    where V0 : KvVertex3Protocol, V0.Math == Math, V1 : KvVertex3Protocol, V1.Math == Math {
        self.init(in: v1.coordinate - v0.coordinate, at: v0.coordinate)
    }


    /// A line matching given ray.
    @inlinable
    public init<V>(_ ray: KvRay3<V>)
    where V : KvVertex3Protocol, V.Math == Math
    {
        self.init(in: ray.front, at: ray.origin.coordinate)
    }


    /// Initializes a line containing given segment.
    @inlinable
    public init<V>(_ segment: KvSegment3<V>)
    where V : KvVertex3Protocol, V.Math == Math {
        let coordinate = segment.endPoints.0.coordinate

        self.init(front: segment.front, origin: coordinate - Math.dot(coordinate, segment.front))
    }



    // MARK: Operations

    /// - Returns: Some coordinate on the receiver.
    @inlinable public var anyCoordinate: Coordinate { origin }


    /// - Returns: *origin* + *step* · *front*.
    @inlinable public func at(_ step: Scalar) -> Coordinate { origin + step * front }


    /// - Returns: The distance from the receiver to given coordinate.
    @inlinable
    public func offset(to x: Coordinate) -> Scalar {
        Math.length(Math.cross(x - origin, front))
    }

    /// - Returns: The distance from the receiver to given vertex.
    @inlinable
    public func offset<V>(to v: V) -> Scalar
    where V : KvVertex3Protocol, V.Math == Math {
        offset(to: v.coordinate)
    }


    /// - Returns: A boolean value indicating whether the receiver contains given coordinate.
    @inlinable
    public func contains(_ c: Coordinate) -> Bool {
        let c = c - origin
        return Math.isZero(Math.cross(front, c), eps: Math.epsArg(front).cross(Math.epsArg(c)).tolerance)
    }

    /// - Returns: A boolean value indicating whether the receiver contains given coordinate.
    @inlinable
    public func contains<V>(_ v: V) -> Bool
    where V : KvVertex3Protocol, V.Math == Math {
        contains(v.coordinate)
    }


    /// - Returns: Receivers coordinate having minimum distance to to given line or *nil* if the receiver and given line are parallel.
    ///
    /// - See  ``projection(of:)``.
    @inlinable
    public func nearby(to rhs: Self) -> Coordinate? {
        let (lFront, lOrigin) = (front, origin)
        let (rFront, rOrigin) = (rhs.front, rhs.origin)

        let n = Math.cross(lFront, rFront)
        let n2 = Math.cross(rFront, n)

        let denominator = Math.dot(lFront, n2)

        guard KvIsNonzero(denominator, eps: Math.epsArg(lFront).dot(Math.epsArg(n2)).tolerance) else { return nil }

        let lt = Math.dot(rFront, Math.cross(n, rOrigin - lOrigin)) / denominator
        // let rt = Math.dot(lFront, Math.cross(n, rOrigin - lOrigin)) / denominator

        return at(lt)
    }


    /// - Returns: Projection of given coordinate on the receiver.
    @inlinable public func projection(of c: Coordinate) -> Coordinate { at(Math.dot((c - origin), front)) }


    /// - Returns: A line produced applying translation by *offset* to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable public func translated(by offset: Vector) -> Self { Self(front: front, origin: origin + offset) }


    /// - Returns: A line produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        Self(front: front, origin: origin * scale)
    }



    // MARK: Operators

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        Self(in: lhs.act(normal: rhs.front), at: lhs.act(coordinate: rhs.origin))
    }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        Self(in: lhs.act(normal: rhs.front), at: lhs.act(coordinate: rhs.origin))
    }

}



// MARK: : KvNumericallyEquatable

extension KvLine3 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(origin, rhs.origin) && Math.isCollinear(front, rhs.front) }

}



// MARK: : Equatable

extension KvLine3 : Equatable { }
