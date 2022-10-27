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

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>



    public let center: Coordinate
    public let radius: Scalar



    /// Memberwise initializer.
    @inlinable
    public init(at center: Coordinate, radius: Scalar) {
        KvDebug.assert(radius.isFinite && KvIsNotNegative(radius), "Invalid argument: unexpected radius (\(radius))")

        self.center = center
        self.radius = radius
    }


    // TODO: Sphere on four coordinates.



    // MARK: Auxiliaries

    /// Sphere equal to zero coordinate. It has zero center and zero radius.
    @inlinable public static var zero: Self { .init(at: .zero, radius: 0) }
    /// Sphere having radius 1 and zero center.
    @inlinable public static var unit: Self { .init(at: .zero, radius: 1) }



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
    

    // TODO: Transformation methods and operators.

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
