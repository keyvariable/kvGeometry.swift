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
//  KvCsgPolygonal.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 27.10.2022.
//

import kvKit

import kvGeometry



/// Protocol for polygonal models.
public protocol KvCsgPolygonal {

    associatedtype Math
    associatedtype Vertex : KvVertex3Protocol where Vertex.Math == Math
    associatedtype Payload


    func traversePolygons(_ body: (KvCsgPolygon3<Vertex, Payload>) throws -> Void) rethrows

}



#if canImport(SceneKit)

import SceneKit



// MARK: Export to SceneKit

extension KvCsgPolygonal {

    /// - Parameter vertexProvider: A callback returning the resulting geometry vertex data. It's passed with the receiver's vertex and a face normal (of arbitrary length) of the polygon.
    ///
    /// - Returns: An instance of *SCNGeometry* containing all the receiver's geometry.
    public func scnGeometry<GeometryVertex : KvSCNGeometrySourceVertex>(
        vertexProvider: (Vertex, Math.Vector3) -> GeometryVertex,
        materialProvider: (Payload) -> SCNMaterial? = { _ in nil }
    ) -> SCNGeometry {
        let geometryBuilder = KvSceneKit.GeometryBuilder<GeometryVertex, Int32>()

        traversePolygons { polygon in
            let baseIndex = geometryBuilder.vertexCount
            let material = materialProvider(polygon.payload)

            let faceNormal = polygon.worldFaceNormal

            polygon.vertices.forEach {
                geometryBuilder.putVertex(vertexProvider($0, faceNormal))
            }

            polygon.forEachTripleOfIndices { i0, i1, i2 in
                geometryBuilder.putTriange(numericCast(baseIndex + i0), numericCast(baseIndex + i1), numericCast(baseIndex + i2), material: material)
            }
        }

        return geometryBuilder.build()
    }

}


extension KvCsgPolygonal where Vertex == KvVertex3<Math, Void> {

    /// - Returns: An instance of *SCNGeometry* containing all the receiver's geometry.
    public func scnGeometry(
        materialProvider: (Payload) -> SCNMaterial? = { _ in nil }
    ) -> SCNGeometry {
        scnGeometry(vertexProvider: { vertex, _ in KvCsgSCNGeometryVertex(underlying: vertex) },
                    materialProvider: materialProvider)
    }

}


extension KvCsgPolygonal where Vertex == KvPosition3<Math, Void> {

    /// - Returns: An instance of *SCNGeometry* containing all the receiver's geometry.
    public func scnGeometry(
        addFaceNormals areFaceNormalsProvided: Bool = false,
        materialProvider: (Payload) -> SCNMaterial? = { _ in nil }
    ) -> SCNGeometry {
        switch areFaceNormalsProvided {
        case false:
            return scnGeometry(vertexProvider: { vertex, _ in KvCsgSCNGeometryPosition(underlying: vertex) },
                               materialProvider: materialProvider)

        case true:
            return scnGeometry(vertexProvider: { vertex, faceNormal in KvCsgSCNGeometryPositionNormal(underlying: vertex, faceNormal: faceNormal) },
                               materialProvider: materialProvider)
        }
    }

}



// MARK: .KvCsgSCNGeometryVertex

internal struct KvCsgSCNGeometryVertex<Math> : KvSCNGeometrySourcePosition3, KvSCNGeometrySourceNormal3, KvSCNGeometrySourceTx0uv
where Math : KvMathScope {

    let underlying: KvVertex3<Math, Void>

    var simdPosition: simd_float3 { simd_float3(underlying.coordinate) }
    var simdNormal: simd_float3 { simd_float3(Math.normalize(underlying.normal)) }
    var simdTx0: simd_float2 { simd_float2(underlying.tx0) }

}



// MARK: .KvCsgSCNGeometryPosition

internal struct KvCsgSCNGeometryPosition<Math> : KvSCNGeometrySourcePosition3
where Math : KvMathScope {

    let underlying: KvPosition3<Math, Void>

    var simdPosition: simd_float3 { simd_float3(underlying.coordinate) }

}



// MARK: .KvCsgSCNGeometryPositionNormal

internal struct KvCsgSCNGeometryPositionNormal<Math> : KvSCNGeometrySourcePosition3, KvSCNGeometrySourceNormal3
where Math : KvMathScope {

    let underlying: KvPosition3<Math, Void>
    let faceNormal: Math.Vector3

    var simdPosition: simd_float3 { simd_float3(underlying.coordinate) }
    var simdNormal: simd_float3 { simd_float3(Math.normalize(faceNormal)) }

}

#endif // canImport(SceneKit)
