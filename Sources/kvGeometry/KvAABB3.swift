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
//  KvAABB3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 22.09.2022.
//

import kvKit



/// Implementation of axis-alligned bounding box in 3D coordinate space.
public struct KvAABB3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Vector = Math.Vector3
    public typealias Coordinate = Vector

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>



    /// The minimum coordinate.
    public var min: Coordinate
    /// The maximum coordinate.
    public var max: Coordinate



    /// Memberwise initializer.
    @inlinable
    public init(min: Coordinate, max: Coordinate) {
        self.min = min
        self.max = max
    }


    /// A bounding box equal to given coordinate.
    @inlinable
    public init(over c: Coordinate) { self.init(min: c, max: c) }

    /// Minimum AABB containing given coordinates.
    @inlinable
    public init(over c0: Coordinate, _ c1: Coordinate) {
        self.init(min: Math.min(c0, c1),
                  max: Math.max(c0, c1))
    }

    /// Minimum AABB containing given coordinates.
    @inlinable
    public init(over c0: Coordinate, _ c1: Coordinate, _ c2: Coordinate) {
        self.init(min: Math.min(Math.min(c0, c1), c2),
                  max: Math.max(Math.max(c0, c1), c2))
    }

    /// Minimum AABB containing given coordinates.
    @inlinable
    public init(over c0: Coordinate, _ c1: Coordinate, _ c2: Coordinate, _ c3: Coordinate, _ rest: Coordinate...) {
        var min = Math.min(Math.min(c0, c1), Math.min(c2, c3))
        var max = Math.max(Math.max(c0, c1), Math.max(c2, c3))

        rest.forEach {
            min = Math.min(min, $0)
            max = Math.max(max, $0)
        }

        self.init(min: min, max: max)
    }

    /// Minimum AABB containing given coordinates.
    @inlinable
    public init?<Coordinates>(over coordinates: Coordinates) where Coordinates : Sequence, Coordinates.Element == Coordinate {
        var iterator = coordinates.makeIterator()

        guard let first = iterator.next() else { return nil }

        var min = first, max = first

        while let c = iterator.next() {
            min = Math.min(min, c)
            max = Math.max(max, c)
        }

        self.init(min: min, max: max)
    }



    // MARK: Auxiliaries

    /// An AABB over the origin coordinate.
    @inlinable public static var zero: Self { Self(over: .zero) }

    @inlinable public static var numberOfVertices: Int { 8 }

    @inlinable public static var vertexIndices: Range<Int> { 0..<8 }



    // MARK: Operations

    @inlinable public var center: Coordinate { Math.mix(min, max, t: 0.5) }

    @inlinable public var size: Vector { max - min }


    /// - Returns: A boolean value indicating whether all the receiver's size elements are positive or zero.
    @inlinable public var isValid: Bool { KvIsNotNegative(size.min()) }
    /// - Returns: A boolean value indicating whether any of the receiver's size elements is zero.
    @inlinable public var isDegenerate: Bool { KvIsZero(Math.abs(size).min()) }


    @inlinable public var vertexIndices: Range<Int> { Self.vertexIndices }

    @inlinable public var coordinates: LazyMapCollection<Range<Int>, Coordinate> { Self.vertexIndices.lazy.map(self.coordinate(at:)) }


    /// - Returns: Coordinate of vertex at given index.
    ///
    /// - SeeAlso: ``vertexIndices``, ``numberOfVertices``
    @inlinable
    public func coordinate(at index: Int) -> Coordinate {
        .init(x: (index & 1) == 0 ? min.x : max.x,
              y: (index & 2) == 0 ? min.y : max.y,
              z: (index & 4) == 0 ? min.z : max.z)
    }


    @inlinable public func contains(_ c: Coordinate) -> Bool {
        KvIs(c.x, in: min.x ... max.x) && KvIs(c.y, in: min.y ... max.y) && KvIs(c.z, in: min.z ... max.z)
    }


    /// Translates the receiver by *offset*.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func translate(by offset: Vector) {
        min += offset
        max += offset
    }


    /// - Returns: An AABB produced applying translation by *offset* to the receiver.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable public func translated(by offset: Vector) -> Self { Self(min: min + offset, max: max + offset) }


    /// Scales the receiver.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public mutating func scale(by scale: Math.Scalar) {
        switch scale >= 0 {
        case true:
            min *= scale
            max *= scale

        case false:
            let oldMin = min
            min = scale * max
            max = scale * oldMin
        }
    }

    /// Scales the receiver.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable public mutating func scale(by scale: Vector) { self = scaled(by: scale) }


    /// - Returns: An AABB produced applying scale to the receiver.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func scaled(by scale: Math.Scalar) -> Self {
        scale >= 0 ? Self(min: min * scale, max: max * scale) : Self(min: max * scale, max: min * scale)
    }

    /// - Returns: An AABB produced applying scale to the receiver.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable public func scaled(by scale: Vector) -> Self { Self(over: min * scale, max * scale) }


    /// Changes the receiver to enclose box produced as result of transformation of the receiver.
    @inlinable public mutating func apply(_ t: Transform) { self = t * self }

    /// Changes the receiver to enclose box produced as result of transformation of the receiver.
    @inlinable public mutating func apply(_ t: AffineTransform) { self = t * self }



    // MARK: Operators

    /// - Returns: An AABB enclosing box produced as result of transformation of the receiver.
    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        Self.init(over: rhs.coordinates.map(lhs.act(coordinate:)))!
    }

    /// - Returns: An AABB enclosing box produced as result of transformation of the receiver.
    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        Self.init(over: rhs.coordinates.map(lhs.act(coordinate:)))!
    }

}



// MARK: : KvNumericallyEquatable, KvNumericallyZeroEquatable

extension KvAABB3 : KvNumericallyEquatable, KvNumericallyZeroEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(min, rhs.min) && Math.isEqual(max, rhs.max)
    }


    /// - Returns: A boolean value indicating whether the receiver is equal to zero coordinate.
    @inlinable public func isZero() -> Bool {
        // - Note: This implementation guarantees that if `isZero() == true` then `isDegenerate == true`, `Math.isZero(min) == true` and `Math.isZero(max) == true`
        return Math.isZero(size) && Math.isZero(center)
    }

}



// MARK: : Equatable

extension KvAABB3 : Equatable { }



// MARK: : Hashable

extension KvAABB3 : Hashable { }
