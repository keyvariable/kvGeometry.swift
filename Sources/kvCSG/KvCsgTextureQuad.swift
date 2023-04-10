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
//  KvCsgTextureQuad.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 11.10.2022.
//

import kvKit

import kvGeometry



/// Quadrilateral on a texture plane.
public struct KvCsgTextureQuad<Math> : ExpressibleByArrayLiteral
where Math : KvMathScope {

    /// Array of 4 2D texture coordinates in CCW order: left-bottom, right-bottom, right-top, left-top.
    public var coordinates: [Math.Vector2]


    /// Left-bottom texture coordinate.
    @inlinable
    public var lb: Math.Vector2 {
        get { coordinates[0] }
        set { coordinates[0] = newValue }
    }
    /// Right-bottom texture coordinate.
    @inlinable
    public var rb: Math.Vector2 {
        get { coordinates[1] }
        set { coordinates[1] = newValue }
    }
    /// Left-top texture coordinate.
    @inlinable
    public var lt: Math.Vector2 {
        get { coordinates[3] }
        set { coordinates[3] = newValue }
    }
    /// Right-top texture coordinate.
    @inlinable
    public var rt: Math.Vector2 {
        get { coordinates[2] }
        set { coordinates[2] = newValue }
    }



    /// Memberwise initializer
    @inlinable
    public init(_ coordinates: [Math.Vector2]) {
        assert(coordinates.count == 4)

        self.coordinates = coordinates
    }


    /// Default initializer.
    @inlinable public init() { self.init(left: 0, right: 0, bottom: 1, top: 0) }


    /// Initializes an instance from array literal.
    @inlinable public init(arrayLiteral elements: Math.Vector2...) { self.init(elements) }


    /// Elementwise initializer.
    @inlinable
    public init(lb: Math.Vector2, rb: Math.Vector2, rt: Math.Vector2, lt: Math.Vector2) {
        self.init([ lb, rb, rt, lt ])
    }


    /// Initializes texture coordinates from given bounds.
    @inlinable
    public init(left: Math.Scalar = 0, right: Math.Scalar = 1, bottom: Math.Scalar = 1, top: Math.Scalar = 0) {
        self.init([
            .init(left, bottom),
            .init(right, bottom),
            .init(right, top),
            .init(left, top),
        ])
    }



    // MARK: Operations

    /// Middle point between the left-bottom and left-top coordinates.
    @inlinable public var left: Math.Vector2 { Math.mix(lb, lt, t: 0.5) }
    /// Middle point between the right-bottom and right-top coordinates.
    @inlinable public var right: Math.Vector2 { Math.mix(rb, rt, t: 0.5) }

    /// Middle point between the left-bottom and right-bottom coordinates.
    @inlinable public var bottom: Math.Vector2 { Math.mix(lb, rb, t: 0.5) }
    /// Middle point between the left-top and right-top coordinates.
    @inlinable public var top: Math.Vector2 { Math.mix(lt, rt, t: 0.5) }

    /// The center point equal to the average of the receiver's coordinates.
    @inlinable public var center: Math.Vector2 { Math.mix(left, right, t: 0.5) }


    /// - Returns: A coordinate transformed from the normalized coordinate space.
    ///
    /// - Note: Normalized coordinate space is the space where the receiver is a square having center at (0.5; 0.5) and of size 1.
    @inlinable
    public func convert(normalized c: Math.Vector2) -> Math.Vector2 {
        Math.mix(Math.mix(lb, rb, t: c.x), Math.mix(lt, rt, t: c.x), t: c.y)
    }

}
