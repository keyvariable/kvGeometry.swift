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
//  KvCsgBox.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 10.10.2022.
//

import Foundation

import kvKit

import kvGeometry



/// Collection of fabrics to generate box shape.
public struct KvCsgBox<Math, Vertex, Payload>
where Math : KvMathScope, Vertex : KvVertex3Protocol, Vertex.Math == Math
{

    public typealias Math = Math
    public typealias Vertex = Vertex
    public typealias Payload = Payload

    public typealias Node = KvCsgBspNode<Math, Vertex, Payload>
    public typealias Vertex3 = KvVertex3<Math, Void>



    // MARK: Private Initializer

    private init() { }



    // MARK: .Configuration

    /// Box parameters
    public struct Configuration {

        /// Half size of a box.
        public var halfSize: Math.Vector3

        /// A boolean value indicating whether the BSP tree contains the balancing plane.
        public var isBalanced: Bool


        /// Memberwise initializer.
        @inlinable
        public init(halfSize: Math.Vector3 = [ 0.5, 0.5, 0.5 ], isBalanced: Bool = false) {
            self.halfSize = halfSize
            self.isBalanced = isBalanced
        }


        @inlinable
        public init(size: Math.Vector3, isBalanced: Bool = false) {
            self.init(halfSize: 0.5 * size, isBalanced: isBalanced)
        }

    }



    // MARK: Fabrics

    /// - Returns: A BSP tree with box geometry having given size and center at the origin.
    @inlinable
    public static func make(
        with configuration: Configuration = .init(),
        textureLayout: TextureLayout = .vertical(),
        payload: Payload,
        vertexBlock: (Vertex3) -> Vertex
    ) throws -> Node {
        typealias Polygon = Node.Polygon3

        let node: Node = bspTree(with: configuration)

        let c0 = Math.Vector3(x: -configuration.halfSize.x, y: -configuration.halfSize.y, z: -configuration.halfSize.z)
        let c1 = Math.Vector3(x:  configuration.halfSize.x, y: -configuration.halfSize.y, z: -configuration.halfSize.z)
        let c2 = Math.Vector3(x: -configuration.halfSize.x, y:  configuration.halfSize.y, z: -configuration.halfSize.z)
        let c3 = Math.Vector3(x:  configuration.halfSize.x, y:  configuration.halfSize.y, z: -configuration.halfSize.z)
        let c4 = Math.Vector3(x: -configuration.halfSize.x, y: -configuration.halfSize.y, z:  configuration.halfSize.z)
        let c5 = Math.Vector3(x:  configuration.halfSize.x, y: -configuration.halfSize.y, z:  configuration.halfSize.z)
        let c6 = Math.Vector3(x: -configuration.halfSize.x, y:  configuration.halfSize.y, z:  configuration.halfSize.z)
        let c7 = Math.Vector3(x:  configuration.halfSize.x, y:  configuration.halfSize.y, z:  configuration.halfSize.z)


        /// - Parameter coordinates: An array of coordinates for the vertices in the same order as *txQuad.coordinates* order: CCW starting at the left-bottom.
        func InsertPolygon(coordinates: [Math.Vector3], normal: Math.Vector3, txQuad: TextureLayout.Quad) throws {
            let vertices = zip(coordinates, txQuad.coordinates).map { vertexBlock(Vertex3($0.0, normal: normal, tx0: $0.1)) }

            guard let polygon = Polygon(vertices, payload: payload)
            else { throw Error.invalidVertices(.init(vertices)) }

            node.insert(polygon)
        }


        // X+
        try InsertPolygon(coordinates: [ c5, c1, c3, c7 ], normal: .unitX, txQuad: textureLayout.px)
        // X–
        try InsertPolygon(coordinates: [ c0, c4, c6, c2 ], normal: .unitNX, txQuad: textureLayout.nx)
        // Y+
        try InsertPolygon(coordinates: [ c6, c7, c3, c2 ], normal: .unitY, txQuad: textureLayout.py)
        // Y–
        try InsertPolygon(coordinates: [ c0, c1, c5, c4 ], normal: .unitNY, txQuad: textureLayout.ny)
        // Z+
        try InsertPolygon(coordinates: [ c4, c5, c7, c6 ], normal: .unitZ, txQuad: textureLayout.pz)
        // Z–
        try InsertPolygon(coordinates: [ c1, c0, c2, c3 ], normal: .unitNZ, txQuad: textureLayout.nz)

        return node
    }



    // MARK: Structure Fabrics

    /// - Returns: A BSP tree without geometry optimized to hold a box having given halfsize and center at the origin. The returned BSP will not split the polygons.
    @inlinable
    public static func bspTree(with configuration: Configuration = .init()) -> Node {
        let dy = -configuration.halfSize.y

        // Y+
        let top = Node(plane: KvPlane3(normal: .unitY, d: dy))

        // Y-
        let bottom = top.setBack(KvPlane3(normal: .unitNY, d: dy))

        if configuration.isBalanced {
            // Diagonal plane: normal is unit Z rotated by 0.5 * .pi
            bottom.setBack(KvPlane3(normal: Math.normalize(Math.Vector3(-configuration.halfSize.z, 0, configuration.halfSize.x)), d: 0))
        }

        return top
    }



    // MARK: .TextureLayout

    /// Texture coordinate rectangles to wrap a box.
    public struct TextureLayout {

        public typealias Quad = KvCsgTextureQuad<Math>


        /// X+ face coordinates.
        public var px: Quad
        /// X– face coordinates.
        public var nx: Quad
        /// Y+ face coordinates.
        public var py: Quad
        /// Y– face coordinates.
        public var ny: Quad
        /// Z+ face coordinates.
        public var pz: Quad
        /// Z– face coordinates.
        public var nz: Quad


        /// Memberwise initializer.
        @inlinable
        public init(px: Quad, nx: Quad, py: Quad, ny: Quad, pz: Quad, nz: Quad) {
            self.px = px
            self.nx = nx
            self.py = py
            self.ny = ny
            self.pz = pz
            self.nz = nz
        }


        // MARK: Fabrics

        /// - Returns: The layout where quads are allinged in contigous vertical line: X+, X–, Y+, Y–, Z+, Z–.
        @inlinable
        public static func vertical(left: Math.Scalar = 0, right: Math.Scalar = 1, bottom: Math.Scalar = 1, top: Math.Scalar = 0) -> Self {
            let y0 = top
            let y1 = Math.mix(top, bottom, t: 1.0 / 6.0)
            let y2 = Math.mix(top, bottom, t: 1.0 / 3.0)
            let y3 = Math.mix(top, bottom, t: 0.5)
            let y4 = Math.mix(top, bottom, t: 2.0 / 3.0)
            let y5 = Math.mix(top, bottom, t: 5.0 / 6.0)
            let y6 = bottom


            return Self(px: .init(left: left, right: right, bottom: y1, top: y0),
                        nx: .init(left: left, right: right, bottom: y2, top: y1),
                        py: .init(left: left, right: right, bottom: y3, top: y2),
                        ny: .init(left: left, right: right, bottom: y4, top: y3),
                        pz: .init(left: left, right: right, bottom: y5, top: y4),
                        nz: .init(left: left, right: right, bottom: y6, top: y5))
        }


        /// - Returns: The layout where quads are allinged in contigous horizontal line: X+, X–, Y+, Y–, Z+, Z–.
        @inlinable
        public static func horizontal(left: Math.Scalar = 0, right: Math.Scalar = 1, bottom: Math.Scalar = 1, top: Math.Scalar = 0) -> Self {
            let x0 = left
            let x1 = Math.mix(left, right, t: 1.0 / 6.0)
            let x2 = Math.mix(left, right, t: 1.0 / 3.0)
            let x3 = Math.mix(left, right, t: 0.5)
            let x4 = Math.mix(left, right, t: 2.0 / 3.0)
            let x5 = Math.mix(left, right, t: 5.0 / 6.0)
            let x6 = right


            return Self(px: .init(left: x0, right: x1, bottom: bottom, top: top),
                        nx: .init(left: x1, right: x2, bottom: bottom, top: top),
                        py: .init(left: x2, right: x3, bottom: bottom, top: top),
                        ny: .init(left: x3, right: x4, bottom: bottom, top: top),
                        pz: .init(left: x4, right: x5, bottom: bottom, top: top),
                        nz: .init(left: x5, right: x6, bottom: bottom, top: top))
        }

    }



    // MARK: .Error

    public enum Error : LocalizedError {

        /// A collection of vertices doesn't produce a convex polygon.
        case invalidVertices(AnySequence<Vertex>)


        // MARK: : LocalizedError

        @inlinable
        public var errorDescription: String? {
            switch self {
            case .invalidVertices(let vertices):
                return "Unable to create a polygon on vertices: \(KvStringKit.with(vertices))"
            }
        }

    }

}



// MARK: where Vertex == KvVertex3<Math, Void>

extension KvCsgBox where Vertex == KvVertex3<Math, Void> {

    /// - Returns: A BSP tree with box geometry having given size and center at the origin.
    @inlinable
    public static func make(with configuration: Configuration = .init(), textureLayout: TextureLayout = .vertical(), payload: Payload) throws -> Node {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { $0 })
    }

}



// MARK: where Vertex == KvPosition3<Math, Void>

extension KvCsgBox where Vertex == KvPosition3<Math, Void> {

    /// - Returns: A BSP tree with box geometry having given size and center at the origin.
    @inlinable
    public static func make(with configuration: Configuration = .init(), textureLayout: TextureLayout = .vertical(), payload: Payload) throws -> Node {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { Vertex($0.coordinate) })
    }

}
