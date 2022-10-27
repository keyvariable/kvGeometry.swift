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
//  KvCSG.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 07.09.2022.
//

import kvKit

import kvGeometry



/// Implementation of constructive solid geometry.
///
/// *Payload* is a type for additional data for polygons that is shared when copying. Use *Void* if unused.
public struct KvCSG<Math, Vertex, Payload>
where Math : KvMathScope, Vertex : KvVertex3Protocol, Vertex.Math == Math
{

    public typealias Math = Math

    /// A type of the shape vertices.
    public typealias Vertex = Vertex
    /// A type for additional data for polygons that is shared when copying. Use *Void* if unused.
    public typealias Payload = Payload

    public typealias Polygon2 = KvCsgPolygon2<Vertex, Payload>
    public typealias Polygon3 = KvCsgPolygon3<Vertex, Payload>

    public typealias Transform = KvCsgTransform<Math>


    /// Box geometry.
    public typealias Box = KvCsgBox<Math, Vertex, Payload>
    /// Cylinder geometry.
    public typealias Cylinder = KvCsgCylinder<Math, Vertex, Payload>
    /// Plane geometry.
    public typealias Plane = KvCsgPlane<Math, Vertex, Payload>



    // MARK: No Initialization

    private init() { }

}
