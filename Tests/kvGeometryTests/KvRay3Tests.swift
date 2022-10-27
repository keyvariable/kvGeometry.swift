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
//  KvRay3Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.09.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvRay3Tests : XCTestCase {

    typealias Vertex<Math : KvMathScope> = KvPosition3<Math, Void>
    typealias Ray<Math : KvMathScope> = KvRay3<Vertex<Math>>



    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Offset to Plane Test

    func testOffsetToPlane() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            let ray = Ray<Math>(in: .one, at: [ 1, 0, 0 ])

            func Assert(_ ray: Ray<Math>, _ plane: KvPlane3<Math>, _ expected: Math.Scalar?) {
                KvAssertEqual(ray.offset(to: plane), expected, by: KvIs(_:equalTo:))
            }

            Assert(ray, .init(normal: .unitX, d: 0), nil)
            Assert(ray, .init(normal: .unitZ, d: 0), 0)
            Assert(ray, .init(normal: .unitX, d: -2), 1)
            Assert(.init(in: .unitX, at: .zero), .init(normal: .unitZ, d: 0), nil)
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }

}
