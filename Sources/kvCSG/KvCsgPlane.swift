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
//  KvCsgPlane.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 20.10.2022.
//

import Foundation

import kvKit

import kvGeometry



/// Collection of fabrics to to handle planes in CSG.
public struct KvCsgPlane<Math, Vertex, Payload>
where Math : KvMathScope, Vertex : KvVertex3Protocol, Vertex.Math == Math
{

    public typealias Math = Math
    public typealias Vertex = Vertex
    public typealias Payload = Payload

    public typealias HalfSpace = KvCsgBspHalfSpace<Math, Vertex, Payload>

    public typealias Vertex3 = KvVertex3<Math, Void>
    public typealias Transform = KvTransform3<Math>

    public typealias Polygon = HalfSpace.Polygon3



    // MARK: Private Initializer

    private init() { }



    // MARK: Fabrics

    /// - Returns: A trivial BSP tree with given polygon.
    @inlinable
    public static func make(with polygon: Polygon) -> HalfSpace {
        HalfSpace(polygons: .init(polygon))
    }



    /// - Returns: A trivial BSP tree wtih a quad geometry descripbed with given *configuration*.
    @inlinable
    public static func make(
        with configuration: QuadConfiguration = .init(),
        textureLayout: TextureLayout,
        payload: Payload,
        vertexBlock: (Vertex3) -> Vertex
    ) throws -> HalfSpace {
        let halfSize = 0.5 * Math.Vector2(configuration.width, configuration.height)

        let coordinates = [
            Math.Vector3(x: -halfSize.x, y: 0, z:  halfSize.y),
            Math.Vector3(x:  halfSize.x, y: 0, z:  halfSize.y),
            Math.Vector3(x:  halfSize.x, y: 0, z: -halfSize.y),
            Math.Vector3(x: -halfSize.x, y: 0, z: -halfSize.y),
        ]

        let input = zip(coordinates, textureLayout.coordinates)

        let vertices: [Vertex] = {
            switch configuration.worldTransform {
            case .none:
                return input.map { vertexBlock(Vertex3($0.0, normal: .unitY, tx0: $0.1)) }
            case .some(let t):
                return input.map { t * vertexBlock(Vertex3($0.0, normal: .unitY, tx0: $0.1)) }
            }
        }()

        guard let polygon = Polygon(vertices, payload: payload)
        else { throw Error.invalidVertices(.init(vertices)) }

        return make(with: polygon)
    }



    // MARK: Structure Fabrics

    /// - Returns: A trivial BSP tree ready to contain a planar geometry on given plane.
    @inlinable
    public static func bspTree(_ plane: KvPlane3<Math>) -> HalfSpace {
        HalfSpace(plane: plane)
    }


    /// - Returns: A trivial BSP tree ready to contain a planar geometry on z = 0 plane transformed by given world transformation.
    @inlinable
    public static func bspTree(_ worldTransform: HalfSpace.Transform) -> HalfSpace {
        HalfSpace(worldTransform: worldTransform)
    }



    // MARK: .QuadConfiguration

    /// A quad in y = 0 plane of given width and height. The center is at the origin. Also the quad is optionally transformed.
    public struct QuadConfiguration {

        public var width: Math.Scalar
        public var height: Math.Scalar

        public var worldTransform: Transform?


        /// Memberwise initializer.
        @inlinable
        public init(width: Math.Scalar = 1, height: Math.Scalar = 1, worldTransform: Transform? = nil) {
            self.width = width
            self.height = height
            self.worldTransform = worldTransform
        }

    }



    // MARK: .TextureLayout

    public typealias TextureLayout = KvCsgTextureQuad<Math>



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

extension KvCsgPlane where Vertex == KvVertex3<Math, Void> {

    /// - Returns: A trivial BSP tree wtih a quad geometry descripbed with given *configuration*.
    @inlinable
    public static func make(
        with configuration: QuadConfiguration = .init(),
        textureLayout: TextureLayout = .init(),
        payload: Payload
    ) throws -> HalfSpace {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { $0 })
    }

}



// MARK: where Vertex == KvPosition3<Math, Void>

extension KvCsgPlane where Vertex == KvPosition3<Math, Void> {

    /// - Returns: A trivial BSP tree wtih a quad geometry descripbed with given *configuration*.
    @inlinable
    public static func make(
        with configuration: QuadConfiguration = .init(),
        textureLayout: TextureLayout = .init(),
        payload: Payload
    ) throws -> HalfSpace {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { Vertex($0.coordinate) })
    }

}
