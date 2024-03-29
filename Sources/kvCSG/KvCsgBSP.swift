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
//  KvCsgBSP.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 30.09.2022.
//

extension KvCSG {

    /// Implementation of BSP tree.
    public struct BSP {

        /// Aliias for ``KvCsgBspNode``.
        public typealias Node = KvCsgBspNode<Math, Vertex, Payload>

        /// Aliias for ``KvCsgBspHalfSpace``.
        public typealias HalfSpace = KvCsgBspHalfSpace<Math, Vertex, Payload>



        // MARK: No Initialization

        private init() { }

    }

}
