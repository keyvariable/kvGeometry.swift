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
//  KvCsgBspHalfSpace.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 27.10.2022.
//

import kvKit

import kvGeometry



/// Implementation of a dedicated BSP node for an infinite shape matching a half space below a plane.
/// Sometimes it's faster and convinient to intersect with a half space shape contining enough geometry then for example with a lagre box.
public class KvCsgBspHalfSpace<Math, Vertex, Payload> : KvCsgPolygonal
where Math : KvMathScope, Vertex : KvVertex3Protocol, Vertex.Math == Math
{
    public typealias Math = Math
    public typealias Vertex = Vertex
    public typealias Payload = Payload

    public typealias Transform = KvCsgTransform<Math>

    public typealias HalfSpace = KvCsgBspHalfSpace
    public typealias Polygon3 = KvCsgPolygon3<Vertex, Payload>

    public typealias Polygons = KvCsgBspNode<Math, Vertex, Payload>.Polygons



    @inlinable public var polygons: Polygons { _polygons }



    /// Memberwise initializer.
    @usableFromInline
    internal required init(polygons: Polygons) {
        _polygons = polygons
    }


    @inlinable
    public convenience init(with polygon: Polygon3) {
        self.init(polygons: Polygons(polygon))
    }


    /// Initializes empty node with given transform.
    @inlinable
    public required convenience init(worldTransform: Transform) {
        self.init(polygons: Polygons(worldTransform: worldTransform))
    }


    /// Initializes empty node with world transformation defined by given plane.
    @inlinable
    public required convenience init(plane: KvPlane3<Math>) {
        self.init(worldTransform: Transform(from: plane))
    }



    @usableFromInline
    internal var _polygons: Polygons



    // MARK: Operations

    @inlinable public var worldTransform: Transform { _polygons.worldTransform }

    /// A boolean value indicating thether the receiver has no geometry.
    @inlinable public var isRedundant: Bool { _polygons.isEmpty }


    /// Transforms the receiver.
    @inlinable public func apply(_ t: KvTransform3<Math>) { _polygons.apply(t) }


    /// Invokes *body* for each polygon of the receiver.
    @inlinable public func traversePolygons(_ body: (Polygon3) throws -> Void) rethrows { try _polygons.forEach(body) }


    /// - Returns: A deep copy of the receiver.
    ///
    /// - Note: It's desinged to save objects when combining.
    @inlinable public func clone() -> Self { Self(polygons: _polygons.clone()) }


    /// Reverses all the geometry.
    @inlinable public func invert() { _polygons.invert() }


    /// Removes the receiver's polygons or their parts inside given BSP tree.
    public func clip<RV, RP>(by rhs: KvCsgBspNode<Math, RV, RP>)
    where RV : KvVertex3Protocol, RV.Math == Math
    {
        // If *rhs* has a back subtree:
        if let rBack = rhs._back {
            // The receiver will contain only front geometry.
            let backPart = dropPolygons(below: rhs.worldTransform)

            if !self.isRedundant, let rFront = rhs._front {
                self.clip(by: rFront)
            }
            // if let b = backPart {
            do {
                backPart.clip(by: rBack)

                // Joining the parts.
                _polygons._ccw.append(contentsOf: backPart._polygons._ccw)
                _polygons._cw.append(contentsOf: backPart._polygons._cw)
            }
        }
        else if let rFront = rhs._front {
            // Removing all the geometry to the back of rhs's plane and coplanar polygons having opposite normals.
            clipPolygons(by: rhs.worldTransform)

            clip(by: rFront)
        }
        else if !rhs._polygons.isEmpty {
            // Removing all the geometry to the back of rhs's plane and coplanar polygons having opposite normals.
            clipPolygons(by: rhs.worldTransform)
        }
    }


    /// Splits (in-place) all geometry in the receiver's subtree (including the receiver) by the z = 0 plane defined by given world transformation.
    ///
    /// - Returns: The split result.
    private func dropPolygons(below t: Transform) -> HalfSpace { HalfSpace(polygons: _polygons.drop(below: t)) }


    /// Clips (in-place) all geometry in the receiver's subtree (including the receiver) below the z = 0 plane defined by given world transformation.
    private func clipPolygons(by t: Transform) { _polygons.clip(by: t) }



    // MARK: Operators

    /// - Returns: A copy of the receiver's subtree where all the nodes are transformed.
    @inlinable public static func *(lhs: KvTransform3<Math>, rhs: HalfSpace) -> HalfSpace { HalfSpace(polygons: lhs * rhs._polygons) }

}
