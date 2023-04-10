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
//  KvSegment3Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.10.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvSegment3Tests : XCTestCase {

    typealias V<Math : KvMathScope> = KvPosition3<Math, Void>
    typealias S<Math : KvMathScope> = KvSegment3<V<Math>>



    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Degenerate Test

    func testDegenerate() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            let v: V<Math> = [ 1, 2, 3 ]
            let s = S<Math>(v, v)

            XCTAssert(s.contains(v))
            XCTAssert(!s.contains(.zero))
            XCTAssert(!s.contains(.one))
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }

}
