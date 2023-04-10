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
//  KvMesh3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 25.09.2022.
//

import kvKit



/// Implementation of a mesh in 3D coordinate space.
public struct KvMesh3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector3
    public typealias Coordinate = Vector



    // MARK: .Volume

    /// Stream accumulating volume of solid body composed of triangles. Volume is calculated as sum of signed pyramid volumes where bases are surface triangles and having the same top vertex.
    public struct Volume : Hashable {

        /// Current volume value.
        @inlinable
        public var value: Scalar {
            _value * ((1.0 as Scalar) / (6.0 as Scalar))      // The common factors ½ (triangle area) and ⅓ (pyramid volume) are applied here.
        }



        /// Default initializer.
        @inlinable public init() { }



        /// - Warning: It's made internal for performance reasons.
        @usableFromInline
        internal private(set) var _value: Scalar = 0


        // MARK: Operations

        /// Process given triangle. The origin is assumed to be at the coordinate center.
        @inlinable
        public mutating func process(_ c₁: Coordinate, _ c₂: Coordinate, _ c₃: Coordinate) {
            _value += Math.dot(Math.cross(c₂ - c₁, c₃ - c₁), c₁)
        }


        /// Process given triangle in shifted coordinate space relative to previously processed triangles.
        @inlinable
        public mutating func process(_ c₁: Coordinate, _ c₂: Coordinate, _ c₃: Coordinate, offset: Vector) {
            _value += Math.dot(Math.cross(c₂ - c₁, c₃ - c₁), c₁ - offset)
        }

    }

}
