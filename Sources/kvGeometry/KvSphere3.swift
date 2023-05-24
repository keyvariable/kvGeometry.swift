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
//  KvSphere3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 25.09.2022.
//

import kvKit



/// Implementation of a sphere in 3D coordinate space.
public struct KvSphere3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector3
    public typealias Coordinate = Vector
    public typealias Quaternion = Math.Quaternion

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>



    public let center: Coordinate
    public let radius: Scalar



    /// Memberwise initializer.
    @inlinable
    public init(at center: Coordinate, radius: Scalar) {
        KvDebug.assert(KvIsNotNegative(radius), "Invalid argument: unexpected radius (\(radius))")

        self.center = center
        self.radius = radius
    }


    /// Circumsphere for two coordinates.
    @inlinable
    public init(_ v1: Coordinate, _ v2: Coordinate) {
        self.init(at: Math.mix(v1, v2, t: 0.5 as Scalar), radius: (0.5 as Scalar) * Math.distance(v1, v2))
    }


    /// Circumsphere for three coordinates if possible.
    public init?(_ v1: Coordinate, _ v2: Coordinate, _ v3: Coordinate) {
        switch InputResult3(v1, v2, v3) {
        case .infine:
            return nil

        case let .segment(start, end):
            self.init(start, end)

        case let .triangle(centerLine):
            let center = centerLine.projection(of: v2)

            self.init(at: center, radius: Math.distance(center, v2))
        }
    }


    /// Circumsphere for four coordinates if possible.
    public init?(_ v1: Coordinate, _ v2: Coordinate, _ v3: Coordinate, _ v4: Coordinate) {
        switch InputResult3(v1, v2, v3) {
        case .infine:
            return nil

        case let .segment(start, end):
            self.init(start, end, v4)

        case let .triangle(centerLine):
            let c = centerLine.projection(of: v2)
            let r2² = Math.distance²(c, v2)

            let plane = KvPlane3<Math>(normal: centerLine.front, at: v2)

            switch plane.contains(v4) {
            case false:
                let r4² = sqr(centerLine.distance(to: v4))
                let h = plane.signedOffset(to: v4)
                // - Note: there is no need to track sign of h. Negative h is compensated by direction of `plane.normal`.
                let offset: Scalar = (0.5 as Scalar) * (h + (abs((r2² - r4²) as Scalar) as Scalar / h))

                let center = c + plane.normal * offset

                self.init(at: center, radius: Math.distance(center, v2))

            case true:
                let radius = r2².squareRoot()

                // Assuming it's better to compare non-squared values to reduce inaccuracy.
                guard KvIs(Math.distance(c, v4), equalTo: radius) else { return nil }

                self.init(at: c, radius: radius)
            }
        }
    }



    // MARK: Auxiliaries

    /// Sphere equal to zero coordinate. It has zero center and zero radius.
    @inlinable public static var zero: Self { .init(at: .zero, radius: 0.0 as Scalar) }
    /// Sphere having radius 1 and zero center.
    @inlinable public static var unit: Self { .init(at: .zero, radius: 1.0 as Scalar) }



    // MARK: Operations

    /// A boolean value indicating whether the receiver has non-negative radius.
    @inlinable public var isValid: Bool { KvIsNotNegative(radius) }

    /// A boolean value indicating whether the receiver is equal to a point in space or has negative radius.
    @inlinable public var isDegenerate: Bool { KvIsNotPositive(radius) }


    /// Coordinate on the receiver's edge having maximum X coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var maxX: Coordinate { center + radius * .unitX }
    /// Coordinate on the receiver's edge having minimum X coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var minX: Coordinate { center - radius * .unitX }
    /// Coordinate on the receiver's edge having maximum Y coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var maxY: Coordinate { center + radius * .unitY }
    /// Coordinate on the receiver's edge having minimum Y coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var minY: Coordinate { center - radius * .unitY }
    /// Coordinate on the receiver's edge having maximum Y coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var maxZ: Coordinate { center + radius * .unitZ }
    /// Coordinate on the receiver's edge having minimum Y coordinate.
    ///
    /// - Note: This property is undefined for invalid spheres.
    @inlinable public var minZ: Coordinate { center - radius * .unitZ }


    /// - Returns: A boolean value indicating whether given coordinate is on the receiver's edge.
    ///
    /// See: ``contains(_:inside:)``.
    @inlinable
    public func contains(_ coordinate: Coordinate) -> Bool {
        KvIs(Math.distance²(center, coordinate), equalTo: radius * radius, eps: squaredDistanceEpsArg(coordinate).tolerance)
    }


    /// - Parameter inside:
    ///     Reference to a boolean value where boolean value is written at.
    ///     The boolean value indicating whether given *coordinate* is inside the receiver but is not on the edge.
    ///
    /// - Returns: A boolean value indicating whether given *coordinate* is on the receiver's edge.
    ///
    /// See: ``contains(_:)``.
    @inlinable
    public func contains(_ coordinate: Coordinate, inside: inout Bool) -> Bool {
        var isOutside: Bool = false
        inside = KvIs(Math.distance²(center, coordinate),
                      lessThan: radius * radius,
                      eps: squaredDistanceEpsArg(coordinate).tolerance,
                      alsoIsGreaterThan: &isOutside)

        return !(inside || isOutside)
    }


    @usableFromInline
    internal func squaredDistanceEpsArg(_ coordinate: Coordinate) -> Math.EpsArg {
        Math.EpsArg3(center).distance²(to: .init(coordinate))
    }


    /// - Returns: A boolean value indicating whether the receiver and given *line* have a coordinate in common.
    @inlinable
    public func intersects(with line: KvLine3<Math>) -> Bool {
        KvIs(line.distance(to: center), lessThanOrEqualTo: radius, eps: line.distanceEpsArg(center).tolerance)
    }


    /// - Returns: A boolean value indicating whether the receiver and given *plane* have a coordinate in common.
    @inlinable
    public func intersects(with plane: KvPlane3<Math>) -> Bool {
        let scale = (1.0 as Scalar) / plane.scale
        return KvIs(plane.offset(to: center) * scale, lessThanOrEqualTo: radius, eps: (plane.epsArg(at: center) * KvEpsArg(scale)).tolerance)
    }


    /// - Returns: A boolean value indicating whether the receiver and given *sphere* have a coordinate in common.
    @inlinable
    public func intersects(with sphere: Self) -> Bool {
        KvIs(Math.distance²(center, sphere.center), lessThanOrEqualTo: radius + sphere.radius, eps: squaredDistanceEpsArg(sphere.center).tolerance)
    }
    


    // MARK: Transformations

    /// - Returns: A sphere produced applying translation by *offset* to all the receiver's ponts.
    @inlinable public func translated(by offset: Vector) -> Self { .init(at: center + offset, radius: radius) }


    /// - Returns: A sphere produced applying scale to all the receiver's ponts.
    @inlinable public func scaled(by scale: Scalar) -> Self { .init(at: center * scale, radius: radius * scale) }


    /// - Returns: A sphere produced applying rotation by given quaterion to all the receiver's ponts.
    @inlinable public func rotated(by quaternion: Quaternion) -> Self { .init(at: quaternion.act(center), radius: radius) }



    // MARK: .InputResult3

    /// Analysis result of 3 coordiate input.
    private enum InputResult3 {

        /// Input produces infinite circumcircle.
        case infine
        /// Input is a non-degenerate tripple of coordinates. Associated line contains center of the circumcircle.
        case triangle(KvLine3<Math>)
        /// Input contains equal coordinates. Associated coordinates are to be used to produce circumcircle.
        case segment(Coordinate, Coordinate)


        init(_ v1: Coordinate, _ v2: Coordinate, _ v3: Coordinate) {
            typealias Plane = KvPlane3<Math>

            guard let n1 = Math.safeNormalize(v1 - v2),
                  let n2 = Math.safeNormalize(v3 - v2)
            else {
                self = .segment(v1, v3)
                return
            }

            let p1 = Plane(normal: n1, at: Math.mix(v2, v1, t: 0.5 as Scalar))
            let p2 = Plane(normal: n2, at: Math.mix(v2, v3, t: 0.5 as Scalar))

            guard let centerLine = p1.intersection(with: p2) else {
                self = Math.isEqual(v1, v3) ? .segment(v1, v2) : .infine
                return
            }

            self = .triangle(centerLine)
        }

    }

}



// MARK: : KvNumericallyEquatable, KvNumericallyZeroEquatable

extension KvSphere3 : KvNumericallyEquatable, KvNumericallyZeroEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(center, rhs.center) && KvIs(radius, equalTo: rhs.radius)
    }


    /// - Returns: A boolean value indicating whether the receiver is equal to zero coordinate.
    @inlinable public func isZero() -> Bool {
        Math.isZero(center) && KvIsZero(radius)
    }

}



// MARK: : Equatable

extension KvSphere3 : Equatable { }



// MARK: : Hashable

extension KvSphere3 : Hashable { }



// MARK: Generaing a Random coordinate

extension KvSphere3 where Scalar.RawSignificand : FixedWidthInteger {

    /// - Returns: A random coordinate on the receiver's edge.
    @inlinable public func randomCoordinate() -> Coordinate { center + Vector.unitRandom() * radius }


    /// - Returns: A random coordinate on the receiver's edge using given *generator*.
    @inlinable
    public func randomCoordinate<G>(using generator: inout G) -> Coordinate
    where G : RandomNumberGenerator
    {
        center + Vector.unitRandom(using: &generator) * radius
    }

}
