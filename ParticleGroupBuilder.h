/*
 *  ParticleGroupBuilder.h
 *  OpenEngine
 *
 *  Created by Peter Kristensen on 02/05/08.
 *  Copyright 2008 Lucky Software. All rights reserved.
 *
 */

#ifndef _PATSYS_PARTICLE_GROUP_BUILDER_H_
#define _PATSYS_PARTICLE_GROUP_BUILDER_H_

#include <Particles/ParticleGroup.h>
#include <Utils/PropertyList.h>
#include <Renderers/IRenderNode.h>

using namespace OpenEngine::Particles;
using namespace OpenEngine::Utils;
using namespace OpenEngine::Renderers;


class ParticleGroupBuilder {
    PropertyList& plist;
    IParticleGroup* pGroup;
    IRenderNode* rNode;
    IEmitter* eMitter;
    string group;
public:
    
    enum FieldType {
        FT_NONE,
        FT_INT,
        FT_FLOAT,
        FT_VECTOR3F,
        FT_VECTOR4F
    };

    ParticleGroupBuilder(PropertyList& plist, string group);
    
    IParticleGroup* GetParticleGroup();
    IRenderNode* GetRenderNode();
    IEmitter* GetEmitter();
    
    void BuildGroup(PropertyList& plist, string group);
    template <class T> static Emitter<T>* BuildEmitter(PropertyList& plist, string group);
    template <class T> static T* BuildParticle(PropertyList& plist, string group);
    template <class T, class G> static IRenderNode* BuildRenderNode(PropertyList& plist, string group, G* g);
    template <class T> static IModifier<T>* BuildModifier(PropertyList& plist, string group);
    
    template <class T> static pair<void (T::*)(float), int> MethodForField_float(string field);
    template <class T> static pair<void (T::*)(Vector<3,float> ), int>
    MethodForField_vec3f(string field);
    template <class T> static pair<Vector<3,float> (T::*)( ), int> MethodForField_get_vec3f(string field);
    //pair<string, void (T::*)(F)> MethodForField(string field);
    static FieldType TypeForField(string field);
};

#endif
