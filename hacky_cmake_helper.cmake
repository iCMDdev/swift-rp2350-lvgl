#[[
Copyright (c) 2024 Raspberry Pi (Trading) Ltd.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
]]

function(gather_vars VAR_NAME PROPERTY_NAME INCLUDED_VAR_NAME LIB)
    #message("${VAR_NAME}:${PROPERTY_NAME}:${INCLUDED_VAR_NAME}:${LIB}")
    if (NOT ${LIB} IN_LIST ${INCLUDED_VAR_NAME})
        string(FIND "${LIB}" "$<" HAS_GENERATOR_EXPRESSIONS)
        string(FIND "${LIB}" "$<" HAS_GENERATOR_EXPRESSIONS)
        if (LIB MATCHES "\\$<")
            message("Skipping generator expression in ${VAR_NAME}: ${LIB}")
        elseif (LIB MATCHES "::@")
            # todo what are these?
        else()
            list(APPEND ${INCLUDED_VAR_NAME} ${LIB})
            set(${INCLUDED_VAR_NAME} ${${INCLUDED_VAR_NAME}} PARENT_SCOPE)
            get_target_property(new_items ${LIB} ${PROPERTY_NAME})
            if (new_items)
                list(APPEND ${VAR_NAME} ${new_items})
            endif()
            get_target_property(trans_depend ${LIB} INTERFACE_LINK_LIBRARIES)
            #            message("Tdep ${trans_depend}")
            if (trans_depend)
                foreach(SUB_LIB ${trans_depend})
                    gather_vars(${VAR_NAME} ${PROPERTY_NAME} ${INCLUDED_VAR_NAME} ${SUB_LIB})
                endforeach()
            endif()
            set(${VAR_NAME} ${${VAR_NAME}} PARENT_SCOPE)
        endif()
    endif()
endfunction()
