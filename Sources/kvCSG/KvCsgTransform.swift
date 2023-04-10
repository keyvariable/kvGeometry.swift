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
//  KvCsgTransform.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 26.10.2022.
//

import kvKit

import kvGeometry



/// Holds transformation with uniform scale component and auxiliary data.
public struct KvCsgTransform<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Underlying = KvTransform3<Math>
    public typealias Plane = KvPlane3<Math>



    /// Direct transformation.
    @inlinable public var direct: Underlying { _direct }
    /// Inserse transformation.
    @inlinable public var inverse: Underlying { _inverse }

    /// The result of the transformationapplied to z = 0 plane.
    @inlinable public var plane: Plane { _plane }



    /// Memberwise initializer.
    @usableFromInline
    internal init(direct: Underlying, inverse: Underlying, plane: Plane) {
        assert(KvCsgTransform.plane(from: direct).isEqual(to: plane))

        _direct = direct
        _inverse = inverse
        _plane = plane
    }


    /// Memberwise initializer with default plane.
    @usableFromInline
    internal init(direct: Underlying, inverse: Underlying) {
        _direct = direct
        _inverse = inverse
        _plane = KvCsgTransform.plane(from: direct)
    }


    /// Initializes an instance with given underlying transformation.
    @inlinable
    public init(_ direct: Underlying) {
        _direct = direct
        _inverse = direct.inverse
        _plane = KvCsgTransform.plane(from: direct)

        assert(!plane.isDegenerate)
    }


    /// Initializes an instance representing given plane.
    @inlinable
    public init(from plane: Plane) {
        assert(!plane.isDegenerate)

        _direct = plane.worldTransform
        _inverse = _direct.inverse
        _plane = plane
    }



    /// Direct transformation.
    @usableFromInline internal var _direct: Underlying
    /// Inserse transformation.
    @usableFromInline internal var _inverse: Underlying

    /// The result of the transformationapplied to z = 0 plane.
    @usableFromInline internal var _plane: Plane



    // MARK: Fabrics

    /// - Returns: Transformation and extracted non-uniform scale transformation component if available.
    @inlinable
    public static func from(_ transform: Underlying) -> (transform: Self, scale: KvAffineTransform3<Math>?) {
        let scale = Math.abs(transform.scale)

        guard !Math.isCollinear(scale, .one)
        else { return (Self(transform), nil) }

        let scaleTransform = KvAffineTransform3<Math>(scale: scale)

        return (Self(transform * scaleTransform.inverse), scaleTransform)
    }

    /// - Returns: Transformation and extracted non-uniform scale transformation component if available.
    @inlinable
    public static func from(_ transform: KvAffineTransform3<Math>) -> (transform: Self, scale: KvAffineTransform3<Math>?) {
        from(Underlying(transform))
    }



    // MARK: Operations

    /// - Returns: A line where the receiver's z= 0 plane intersects z = 0 plane of given transformation. Returned line is in the receiver's local coordinate space.
    @inlinable
    public func localPlaneIntersection(withPlaneFrom rhs: Self) -> KvLine2<Math>? {
        let plane = _inverse * rhs.plane

        // - Note: Optimized intersection plane.intersection(with: [ 0, 0, 1, 0 ])

        let n = Math.make2(plane.normal)
        let front = Math.Vector2(n.y, -n.x)

        guard Math.isNonzero(front) else { return nil }

        return KvLine2<Math>(in: front, at: (-plane.d / Math.length²(n)) * n)
    }



    // MARK: Operators

    @inlinable public static func *(lhs: Self, rhs: Self) -> Self { Self(direct: lhs.direct * rhs.direct, inverse: rhs.inverse * lhs.inverse) }



    // MARK: Auxiliaries

    @inlinable public static var identity: Self { Self(direct: .identity, inverse: .identity, plane: [ 0, 0, 1, 0 ]) }


    /// - Returns: Plane defined by given direct transformation.
    @inlinable
    public static func plane(from direct: Underlying) -> Plane {
        Plane(normal: direct.act(normal: .unitZ), at: direct.translation)
    }

}



// MARK: :

extension KvCsgTransform : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver is numerically equal to *rhs*.
    @inlinable public func isEqual(to rhs: Self) -> Bool { _direct.isEqual(to: rhs._direct) }

}
