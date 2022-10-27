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
//  KvCsgCylinder.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 11.10.2022.
//

import Foundation

import kvKit

import kvGeometry



/// Collection of fabrics to generate cylinder shape. Generated cylinders have base faces parallel to the local XZ plane.
public struct KvCsgCylinder<Math, Vertex, Payload>
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

    /// Cylinder parameters.
    public struct Configuration {

        public var radius: Math.Scalar
        public var height: Math.Scalar

        /// The number of subdivisions around the circumference of the cylinder.
        public var radialSegmentCount: Int
        /// The number of subdivisions in the sides of the cylinder along Y-axis.
        public var heightSegmentCount: Int

        /// Maxmum number of the BSP tree levels with balancing planes.
        public var radialLevelLimit: Int


        /// Memberwise initializer
        @inlinable
        public init(radius: Math.Scalar = 0.5,
                    height: Math.Scalar = 1,
                    radialSegmentCount: Int = 24,
                    heightSegmentCount: Int = 1,
                    radialLevelLimit: Int = .max)
        {
            self.radius = radius
            self.height = height
            self.radialSegmentCount = radialSegmentCount
            self.heightSegmentCount = heightSegmentCount
            self.radialLevelLimit = radialLevelLimit
        }

    }



    // MARK: Fabrics

    /// - Returns: A BSP tree with polygons optimized to hold a cylinder with given *configuration*.
    @inlinable
    public static func make(
        with configuration: Configuration = .init(),
        textureLayout: TextureLayout = .init(),
        payload: Payload,
        vertexBlock: (Vertex3) -> Vertex
    ) throws -> Node {
        typealias Polygon = Node.Polygon3

        let py = 0.5 * configuration.height, ny = -py
        let (node, sideNormals) = bspTreeAndSideNormals(with: configuration)


        /// - Returns: Coordinate where
        func TxBase(_ quad: TextureLayout.Quad, in direction: Math.Vector2) -> Math.Vector2 {
            quad.convert(normalized: direction * 0.5 + 0.5)
        }


        func InsertPolygon<Vertices>(with vertices: Vertices) throws
        where Vertices : Sequence, Vertices.Element == Vertex {
            guard let polygon = Polygon(vertices, payload: payload)
            else { throw Error.invalidVertices(.init(vertices)) }

            node.insert(polygon)
        }


        /// Vertices of Y+ face.
        let pyVertices: [Vertex]
        /// Vertices of Y– face.
        let nyVertices: [Vertex]

        (pyVertices, nyVertices) = sideNormals.reduce(into: (.init(), .init()), { partialResult, normal in
            let c2 = configuration.radius * normal

            partialResult.0.append(vertexBlock(
                Vertex3(Math.Vector3(c2.x, py, c2.y),
                        normal: .unitY,
                        tx0: TxBase(textureLayout.py, in: normal))
            ))
            partialResult.1.append(vertexBlock(
                Vertex3(
                    Math.Vector3(c2.x, ny, c2.y),
                    normal: .unitNY,
                    tx0: TxBase(textureLayout.ny, in: normal)
                )
            ))
        })

        try InsertPolygon(with: pyVertices.reversed())
        try InsertPolygon(with: nyVertices)

        // Side polygons
        if configuration.heightSegmentCount > 0 {
            /// Factor to convert normal index to normalized texture U coordinate.
            let indexToTu = 1 / Math.Scalar(sideNormals.count)
            /// Factor to convert vertical vertex index to normalized texture V coordinate.
            let indexToTv = 1 / Math.Scalar(configuration.heightSegmentCount)


            typealias Segment = [Vertex]


            func MakeSegment(at index: Int) -> Segment {
                let normal = { Math.Vector3($0.x, 0, $0.y) }(sideNormals[index])
                let tu = Math.Scalar(index) * indexToTu

                return (0...configuration.heightSegmentCount).map { vIndex in
                    let tv = Math.Scalar(vIndex) * indexToTv

                    let coordinate = Math.mix(nyVertices[index].coordinate, pyVertices[index].coordinate, t: tv)
                    let tx0 = textureLayout.side.convert(normalized: .init(tu, tv))

                    return vertexBlock(Vertex3(coordinate, normal: normal, tx0: tx0))
                }
            }


            func Insert(left: Segment, right: Segment) throws {
                var prevLeft = left[0]
                var prevRight = right[0]

                try zip(left.dropFirst(), right.dropFirst()).forEach { (nextLeft, nextRight) in
                    defer {
                        prevLeft = nextLeft
                        prevRight = nextRight
                    }

                    try InsertPolygon(with: [ nextLeft, nextRight, prevRight, prevLeft ])
                }
            }


            var iterator = sideNormals.indices.makeIterator()

            guard let first = iterator.next().map(MakeSegment(at:)),
                  var prev = iterator.next().map(MakeSegment(at:))
            else { throw Error.invalidRadialSegmentCount(configuration.radialSegmentCount) }

            try Insert(left: first, right: prev)

            while let next = iterator.next().map(MakeSegment(at:)) {
                defer { prev = next }

                try Insert(left: prev, right: next)
            }

            try Insert(left: prev, right: first)
        }

        return node
    }



    // MARK: Structure Fabrics

    /// - Returns: A BSP tree without polygons optimized to hold a cylinder with given *configuration*. The returned BSP will not split the polygons.
    @inlinable
    public static func bspTree(with configuration: Configuration = .init()) -> Node {
        bspTreeAndSideNormals(with: configuration).node
    }


    /// Internal implementation returning the root node and auxiliary side normal vectors.
    @usableFromInline
    internal static func bspTreeAndSideNormals(with configuration: Configuration) -> (node: Node, sideNormals: [Math.Vector2]) {
        let sideNormals: [Math.Vector2]
        do {
            let dAngle = 2 * Math.Scalar.pi / Math.Scalar(configuration.radialSegmentCount)

            sideNormals = (0 ..< configuration.radialSegmentCount).map {
                let (sine, cosine) = Math.sincos(Math.Scalar($0) * dAngle)
                return Math.Vector2(x: cosine, y: sine)
            }
        }


        func SetRadialPlanes(_ indexRange: Range<Int>, limit: Int, _ subnodeBlock: (KvPlane3<Math>) -> Node) {
            guard limit > 0 else { return }

            let middle = (indexRange.upperBound + indexRange.lowerBound) >> 1

            // - Note: It's true only when .upperBound > .lowerBound + 1
            guard middle != indexRange.lowerBound else { return }

            let v = sideNormals[middle]
            let subnode = subnodeBlock(.init(normal: .init(x: -v.y, y: 0, z: v.x), d: 0))

            let nextLimit = limit - 1

            SetRadialPlanes(indexRange.lowerBound ..< middle, limit: nextLimit, subnode.setBack(_:))
            SetRadialPlanes(middle ..< indexRange.upperBound, limit: nextLimit, subnode.setFront(_:))
        }


        let dy = -0.5 * configuration.height

        // Y+
        let top = Node(plane: KvPlane3(normal: .unitY, d: dy))

        // Y-
        let bottom = top.setBack(KvPlane3(normal: .unitNY, d: dy))

        SetRadialPlanes(sideNormals.indices, limit: configuration.radialLevelLimit, bottom.setBack(_:))

        return (top, sideNormals)
    }



    // MARK: .TextureLayout

    /// Texture coordinate rectangles to wrap a cylinder.
    public struct TextureLayout {

        public typealias Quad = KvCsgTextureQuad<Math>


        /// Coordinate bounding quad for *Y+* face.
        public var py: Quad
        /// Coordinate bounding quad for *Y–* face.
        public var ny: Quad
        /// Side coordinates.
        public var side: Quad


        /// Memberwise initializer.
        @inlinable
        public init(
            py: Quad = .init(left: 0.0, right: 0.5, bottom: 1.0, top: 0.5),
            ny: Quad = .init(left: 0.5, right: 1.0, bottom: 1.0, top: 0.5),
            side: Quad = .init(left: 0.0, right: 1.0, bottom: 0.5, top: 0.0)
        ) {
            self.py = py
            self.ny = ny
            self.side = side
        }

    }



    // MARK: .Error

    public enum Error : LocalizedError {

        case invalidRadialSegmentCount(Int)
        /// A collection of vertices doesn't produce a convex polygon.
        case invalidVertices(AnySequence<Vertex>)


        // MARK: : LocalizedError

        @inlinable
        public var errorDescription: String? {
            switch self {
            case .invalidRadialSegmentCount(let value):
                return "Not enought side segments (\(value)) to produce a solid cylinder"
            case .invalidVertices(let vertices):
                return "Unable to create a polygon on vertices: \(KvStringKit.with(vertices))"
            }
        }

    }

}



// MARK: where Vertex == KvVertex3<Math, Void>>

extension KvCsgCylinder where Vertex == KvVertex3<Math, Void> {

    /// - Returns: A BSP tree with polygons optimized to hold a cylinder with given *configuration*.
    @inlinable
    public static func make(
        with configuration: Configuration = .init(),
        textureLayout: TextureLayout = .init(),
        payload: Payload
    ) throws -> Node {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { $0 })
    }

}



// MARK: where Vertex == KvPosition3<Math, Void>

extension KvCsgCylinder where Vertex == KvPosition3<Math, Void> {

    /// - Returns: A BSP tree with polygons optimized to hold a cylinder with given *configuration*.
    @inlinable
    public static func make(
        with configuration: Configuration = .init(),
        textureLayout: TextureLayout = .init(),
        payload: Payload
    ) throws -> Node {
        try make(with: configuration, textureLayout: textureLayout, payload: payload, vertexBlock: { Vertex($0.coordinate) })
    }

}
