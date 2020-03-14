//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author(s):	James Stanard
//				Alex Nankervis
//
// Thanks to Michal Drobot for his feedback.

#include "ModelViewerRS.hlsli"
#include "LightGrid.hlsli"

// outdated warning about for-loop variable scope
#pragma warning (disable: 3078)
// single-iteration loop
#pragma warning (disable: 3557)

#define SINGLE_SAMPLE 1;

struct VSOutput
{
    sample float4 position : SV_Position;
    sample float3 worldPos : WorldPos;
    sample float2 uv : TexCoord0;
    sample float3 viewDir : TexCoord1;
    sample float3 shadowCoord : TexCoord2;
    sample float3 normal : Normal;
    sample float3 tangent : Tangent;
    sample float3 bitangent : Bitangent;
};

Texture2D<float3> texDiffuse		: register(t0); 
Texture2D<float3> texSpecular		: register(t1);
//Texture2D<float4> texEmissive		: register(t2);
Texture2D<float3> texNormal			: register(t3);
//Texture2D<float4> texLightmap		: register(t4); // ambient or SSAO baked light map 
//Texture2D<float4> texReflection	: register(t5);
Texture2D<float> texSSAO			: register(t64);
Texture2D<float> texShadow			: register(t65);

StructuredBuffer<LightData> lightBuffer : register(t66);
Texture2DArray<float> lightShadowArrayTex : register(t67);
ByteAddressBuffer lightGrid : register(t68);
ByteAddressBuffer lightGridBitMask : register(t69);

cbuffer PSConstants : register(b0)
{
	float3 SunDirection; // normalized on input
	float3 Isun; // sun light intensity
	float3 Ia;

	float3 mtlAmbientColor; // ambient strength reflected back (used)
	float3 mtlDiffuseColor; // surface diffusion reflected back (used)
	float3 mtlSpecularColor; // gloss color (used)
	float3 mtlEmissiveColor; // radiant
	float3 mtlTransparentColor; // light passing through a transparent surface is multiplied by this filter color
	float mtlIndexOfRefraction; // 0.001 to 10; 1.0 means that light does not bend, glass has value 1.5
	float mtlSpecularStrength; // exponent 0 to 1000, shininess / gloss (used)
	float mtlDissolved; // 0 transparent to 1 opaque

	float2 InvTileDim;
	uint2 TileCount;
	uint2 FirstLightIndex; // [0] first cone light, [1] first cone shadowed light
	uint FrameIndexMod2; // not used in PS ???? FrameIndex
	float ShadowTexelSize; // 1.0 / g_ShadowBuffer.GetWidth()
}

SamplerState sampler0 : register(s0);
SamplerComparisonState shadowSampler : register(s1);

float3 NormalSampleToWorldSpace(float3 normalMapSample, float3 unitNormalW, float3 tangentW)
{
    // done before calling, so omit
    //float3 normalT = 2.0 * normalMapSample - 1.0f;

    float3 N = unitNormalW;
    float3 T = normalize(tangentW - dot(tangentW, N)*N);
    float3 B = cross(N,T); // right hand or left hand????

    float3x3 TBN = float3x3(T, B, N);

    float3 bumpedNormalW = mul(N, TBN); // order changes direction of lighting !!!!!!
    return bumpedNormalW;
}

void AntiAliasSpecular( inout float3 texNormal, inout float gloss )
{
    float normalLenSq = dot(texNormal, texNormal);
    float invNormalLen = rsqrt(normalLenSq);
    texNormal *= invNormalLen;
    gloss = lerp(1, gloss, rcp(invNormalLen));
}

// Apply fresnel to modulate the specular albedo
void FSchlick( inout float3 specular, inout float3 diffuse, float3 pointTolight, float3 halfVec )
{
    float fresnel = pow(1.0 - saturate(dot(pointTolight, halfVec)), 5.0);
    specular = lerp(specular, 1, fresnel);
    diffuse = lerp(diffuse, 0, fresnel);
}

float GetShadow( float3 ShadowCoord )
{
#ifdef SINGLE_SAMPLE
    float result = texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy, ShadowCoord.z );
#else
    const float Dilation = 2.0;
    float d1 = Dilation * ShadowTexelSize.x * 0.125;
    float d2 = Dilation * ShadowTexelSize.x * 0.875;
    float d3 = Dilation * ShadowTexelSize.x * 0.625;
    float d4 = Dilation * ShadowTexelSize.x * 0.375;
    float result = (
        2.0 * texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy, ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2(-d2,  d1), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2(-d1, -d2), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2( d2, -d1), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2( d1,  d2), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2(-d4,  d3), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2(-d3, -d4), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2( d4, -d3), ShadowCoord.z ) +
        texShadow.SampleCmpLevelZero( shadowSampler, ShadowCoord.xy + float2( d3,  d4), ShadowCoord.z )
        ) / 10.0;
#endif
    return result * result;
}

float GetShadowConeLight(uint lightIndex, float3 shadowCoord)
{
    float result = lightShadowArrayTex.SampleCmpLevelZero(
        shadowSampler, float3(shadowCoord.xy, lightIndex), shadowCoord.z);
    return result * result;
}

// https://en.wikipedia.org/wiki/Blinn�Phong_shading_model
float3 ApplyLightCommon(
    float3	diffuseColor,	// Diffuse albedo
    float3	specularColor,	// Specular albedo
    float	specularMask,	// Where is it shiny or dingy?
    float	gloss,			// Specular power
    float3	normal,			// World-space normal
    float3	fromEye,		// World-space vector from eye to point
    float3	pointTolight,	// World-space vector from point to light
    float3	lightColor		// Radiance of directional light
    )
{
	/*
	 H	unit vector bisector between L and V
	 L	unit light vector
	 N	unit surface normal
	 V	unit view vector
     */
    float3 L = pointTolight; // light rays travel opposite direction to sun
    float3 H = normalize(L + fromEye);
    float nDotH = max(dot(normal, H), 0.0f); // specAngle
    float nDotL = max(dot(normal, L), 0.0f);
 
	float specular = pow(nDotH, gloss);

    FSchlick( diffuseColor, specularColor, L, H ); // modifies diffuse and specular

	return nDotL * lightColor * (diffuseColor + specularMask * specular * specularColor);
}

float3 ApplyDirectionalLight(
    float3	diffuseColor,	// Diffuse albedo
    float3	specularColor,	// Specular albedo
    float	specularMask,	// Where is it shiny or dingy?
    float	gloss,			// Specular power
    float3	normal,			// World-space normal
    float3	viewDir,		// World-space vector from eye to point
    float3	pointTolight,   // World-space vector from point to light
    float3	lightColor,		// Radiance of directional light
    float3	shadowCoord		// Shadow coordinate (Shadow map UV & light-relative Z)
    )
{
    float shadow = GetShadow(shadowCoord);

    return shadow * ApplyLightCommon(
        diffuseColor,
        specularColor,
        specularMask,
        gloss,
        normal,
        viewDir,
        pointTolight,
        lightColor
        );
}

float3 ApplyPointLight(
    float3	diffuseColor,	// Diffuse albedo
    float3	specularColor,	// Specular albedo
    float	specularMask,	// Where is it shiny or dingy?
    float	gloss,			// Specular power
    float3	normal,			// World-space normal
    float3	viewDir,		// World-space vector from eye to point
    float3	worldPos,		// World-space fragment position
    float3	lightPos,		// World-space light position
    float	lightRadiusSq,
    float3	lightColor		// Radiance of directional light
    )
{
    float3 lightDir = lightPos - worldPos;
    float lightDistSq = dot(lightDir, lightDir);
    float invLightDist = rsqrt(lightDistSq);
    lightDir *= invLightDist;

    // modify 1/d^2 * R^2 to fall off at a fixed radius
    // (R/d)^2 - d/R = [(1/d^2) - (1/R^2)*(d/R)] * R^2
    float distanceFalloff = lightRadiusSq * (invLightDist * invLightDist);
    distanceFalloff = max(0, distanceFalloff - rsqrt(distanceFalloff));

    return distanceFalloff * ApplyLightCommon(
        diffuseColor,
        specularColor,
        specularMask,
        gloss,
        normal,
        viewDir,
        lightDir,
        lightColor
        );
}

float3 ApplyConeLight(
    float3	diffuseColor,	// Diffuse albedo
    float3	specularColor,	// Specular albedo
    float	specularMask,	// Where is it shiny or dingy?
    float	gloss,			// Specular power
    float3	normal,			// World-space normal
    float3	viewDir,		// World-space vector from eye to point
    float3	worldPos,		// World-space fragment position
    float3	lightPos,		// World-space light position
    float	lightRadiusSq,
    float3	lightColor,		// Radiance of directional light
    float3	coneDir,
    float2	coneAngles
    )
{
    float3 lightDir = lightPos - worldPos;
    float lightDistSq = dot(lightDir, lightDir);
    float invLightDist = rsqrt(lightDistSq);
    lightDir *= invLightDist;

    // modify 1/d^2 * R^2 to fall off at a fixed radius
    // (R/d)^2 - d/R = [(1/d^2) - (1/R^2)*(d/R)] * R^2
    float distanceFalloff = lightRadiusSq * (invLightDist * invLightDist);
    distanceFalloff = max(0, distanceFalloff - rsqrt(distanceFalloff));

    float coneFalloff = dot(-lightDir, coneDir);
    coneFalloff = saturate((coneFalloff - coneAngles.y) * coneAngles.x);

    return (coneFalloff * distanceFalloff) * ApplyLightCommon(
        diffuseColor,
        specularColor,
        specularMask,
        gloss,
        normal,
        viewDir,
        lightDir,
        lightColor
        );
}

float3 ApplyConeShadowedLight(
    float3	diffuseColor,	// Diffuse albedo
    float3	specularColor,	// Specular albedo
    float	specularMask,	// Where is it shiny or dingy?
    float	gloss,			// Specular power
    float3	normal,			// World-space normal
    float3	viewDir,		// World-space vector from eye to point
    float3	worldPos,		// World-space fragment position
    float3	lightPos,		// World-space light position
    float	lightRadiusSq,
    float3	lightColor,		// Radiance of directional light
    float3	coneDir,
    float2	coneAngles,
    float4x4 shadowTextureMatrix,
    uint	lightIndex
    )
{
    float4 shadowCoord = mul(shadowTextureMatrix, float4(worldPos, 1.0));
    shadowCoord.xyz *= rcp(shadowCoord.w);
    float shadow = GetShadowConeLight(lightIndex, shadowCoord.xyz);

    return shadow * ApplyConeLight(
        diffuseColor,
        specularColor,
        specularMask,
        gloss,
        normal,
        viewDir,
        worldPos,
        lightPos,
        lightRadiusSq,
        lightColor,
        coneDir,
        coneAngles
        );
}

// options for F+ variants and optimizations
#ifdef _WAVE_OP // SM 6.0 (new shader compiler)

// choose one of these:
//# define BIT_MASK
# define BIT_MASK_SORTED
//# define SCALAR_LOOP
//# define SCALAR_BRANCH

// enable to amortize latency of vector read in exchange for additional VGPRs being held
# define LIGHT_GRID_PRELOADING

// configured for 32 sphere lights, 64 cone lights, and 32 cone shadowed lights
# define POINT_LIGHT_GROUPS			1
# define SPOT_LIGHT_GROUPS			2
# define SHADOWED_SPOT_LIGHT_GROUPS	1
# define POINT_LIGHT_GROUPS_TAIL			POINT_LIGHT_GROUPS // 1
# define SPOT_LIGHT_GROUPS_TAIL				POINT_LIGHT_GROUPS_TAIL + SPOT_LIGHT_GROUPS // 1 + 2 = 3
# define SHADOWED_SPOT_LIGHT_GROUPS_TAIL	SPOT_LIGHT_GROUPS_TAIL + SHADOWED_SPOT_LIGHT_GROUPS // 3 + 1 = 4


uint GetGroupBits(uint groupIndex, uint tileIndex, uint lightBitMaskGroups[4])
{
#ifdef LIGHT_GRID_PRELOADING
    return lightBitMaskGroups[groupIndex];
#else
    return lightGridBitMask.Load(tileIndex * 16 + groupIndex * 4);
#endif
}

uint64_t Ballot64(bool b)
{
    uint4 ballots = WaveActiveBallot(b);
    return (uint64_t)ballots.y << 32 | (uint64_t)ballots.x;
}

#endif // _WAVE_OP

// Helper function for iterating over a sparse list of bits.  Gets4 ,*(4 ,*(4 3(4 *(4 .	(4 3( 4 *(,4 s� 

rn�po� 
&(�3 o� 
&rd�po� 
&(
4 ,rx�po� 
&o� 
(-4 *  0 �   _ sy 

(4 (!4 r~�po� 
&(�3 o�2 o�2 .Ror4 o�2 io�3 	,<	o�3 -4r4�po� 
&	o�3 �U  o� 
o� 
&r�)po� 
&(�3 o�2 o�2 .r��po� 
&o�2 o�2 .r��po� 
&rd�po� 
&o� 
(-4 *0 %   1  (,4 s� 

r�po� 
&o� 
(-4 *   0    1  sy 

r"�po� 
&o� 
(-4 * 0 %   1  (,4 s� 

rJ�po� 
&o� 
(-4 *   0 -   K ($4 
($4 ("4 Xo$4 1o$4 ��**   0 ^   ` (&4 
(4 .*(�3 -*o$4 Xo$4 o"4 X+	oG4 3*o
4 ,*	Y	1�*  0 b   a (&4 
sF4 (4 3I($4 X($4 ("4 X+,oG4 o4 3o�3 3oH4 X	1�*  0 b   a (&4 
sF4 (4 3I($4 X($4 ("4 X+,oG4 o4 3o�3 3oH4 X	1�*  0 m   b (&4 
(4 3X(�3 ,Lo$4 Xo$4 o"4 X+-	oG4 .$o4 3o�3 3X	X	1�*   0 l  c (&4 
(4 sC4 8C  	oG4 o4 o�2 j8  o�3 	j
oj3 :�   8�   oj 
t� o=4 o[3 3oA4 -
3oB4 	+fo=4 o[3 1:s<4 o@4 o[3 o>4 
�oB4 o 
	+o=4 
Xoh 
?_���	-4s<4 o@4 o[3 o>4 
�oB4 og 
&o[3 Xoz3 ?����	X	oh 
?����*�{�$ ./{�$ .&{�$ .{�$ .{�$ .
{�$ �** 0 f       {�$ .[{�$ 	.Q{�$ 
.G{�$ .={�$ .3{�$ .){�$ .{�$ .{�$ .{�$ �***{�$ �*�{�$ .{�$ .{�$ .{�$ �**   0 +   2  (4 ,*{�$ 3(,4 o� 

o� 
�**J($4 2{�$ **"}�$ *{�$ *"}�$ *N(4 3{�$ **"}�$ *�($4 /*(4 3*(4 ,
(4 -*(4 -**{�$ *"}�$ *V("4 0
(,4 �**{�$ *"}�$ *{�$ *"}�$ *{�$ *{�$ *"}�$ *   0 3   S (*4 
,o4 3o*4 
(4 .-(�2 *o4 *{�$ *22}�$ *{�$ *"}�$ *{�$ *"}�$ *:($4 ("4 X*{�$ *�{�$ -(&4 ,(&4 oW4 *{�$ *"}�$ *{�$ *"}�$ *{�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ * 0 �   d (	4 ,(4 o�2 *(4 3%(4 o�2 ,(4 o�2 o�3 *(64 *(4 	.	(4 3(4 o�2 *(*4 
+#o4 .
	.3o54 *o*4 
-�*Z(*4 ,(*4 o54 **J(54 (64 ��*�(64 .(4 o�2 *(4 o�2 *�(64 3(4 o�2 *(4 o�2 *�(64 .(4 o�2 *(4 o�2 *   0 >      �� %r�  p�%r�  p�%rt�p�%r~�p�%r��p�%r��p�%rl�p�%r��p�%r��p�%	r��p�%
r��p�%r��p���$ �� %�M; (\ 
��$ �� %r�  p�%r�  p�%rQ� p�%rO� p�%r� p�%rs� p�%r߫ p�%ry� p�%rǪ p�%	r�� p�%
r� p�%rY� p�%r�� p�%r�� p�%re� p�%r� p�%r7� p�%r� p�%r��p�%r��p���$ *v(� 
j}�$ }�$ }�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *6oj 
t� *   0 Z   e 
+I(D4 oA4 -52o?4 o$4 1#o?4 o4 o�2 o�3 -o?4 o$4 
Xoh 
2�*^d(q 
}�$ }�$ *6oj 
t� *:oh 
(d4 * 0 !   S (g4 
oh 
1oh 
Y(f4 *   0 '   f oh 
Y
+(G4 o4 ,*Y
/�*�oh 
1oh 
Y(G4 o4 �**  0 �  g sF4 
(G4 o"4 	Xoh 
2oh 
YY	X,X8S  (G4 o	4 9�   o74 9�   o)4 9�   X+M(G4 		o	4 ,@	o4 .6	o4 o�2 o4 o�2 3	o)4 o)4 3X1�Y>�   s�3 

o*4 o4 s�2 o 4 
o4 o4 o�2 o�2 o)4 
(e4 X+bo4 
3
(i4 +Mo4 	3
(j4 +8o4 3og 
&(k4 X+o4 3
(l4 X>����,(m4 *   0   h oh 
//*(L4 (G4 
o"4 Xoh 
2oh 
YYX8�   	(G4 o"4 -o�3 8�   o�3 o,4 s� 
o"4 	X	X+(G4 		o,4 o� 
&X1�o� 
o-4 o�3 o4 	X(f4 Y(R4 o14 -~� 
o-4 	Y	<<���*0 
  h oh 
//*(L4 (G4 
o"4 Xoh 
2oh 
YYX8�   	(G4 o"4 -	.o�3 8�   o"4 >�   	.o�3 o,4 s� 
o"4 	X	X+(G4 		o,4 o� 
&X1�o� 
o-4 	.o�3 o4 	X(f4 Y	Y	<>���*  0    	  
+(M4 X
oh 
2�*  0 f   i oh 
/2Xoh 
2*(G4 
o4 -*o4 o#4 Xo"4 X+(G4 	o+4 	o"4 XX1�*  0 s   j oh 
//*(G4 
o4 -*(R4 (S4 oh 
Y+(G4 o4 ,(Q4 Y0�oh 
YY(P4 (R4 (S4 * 0 G   k (� 
,?
+2(G4 X+Xo(4 1�o*4 +	o*4 	-�X
oh 
2�* 0 L   l (� 
,D
+7(G4 o*4 o4 	
YE                       X
oh 
2�*0 (   	  
+(G4 o4 ,(Q4 *X
oh 
2�*0 ]   m 
{�$ ,Q{�$ ob4 {�$ oh 
Y+2{�$ oG4 o4 ,o4 3X
+
o4 .Y/�*&(U4 * 0 R   f {�$ ,H{�$ ob4 {�$ oh 
Y
+){�$ oG4 o4 ,o$4 o$4 /*Y
/�*  0 }   f {�$ ,s{�$ ob4 {�$ oh 
Y
+T{�$ oG4 o4 ,;o4 .o4 .o4 .
o4 3o4 *o4 3*Y
/�*   0 �   n s�3 
{�$ 9�   {�$ ob4 +'{�$ oG4 	o4 ,	o4 3XX{�$ oh 
2�+f{�$ oG4 o4 ,Ho4 	3=o4 o�2 o4 o�2 o4 o�2 o4 o�2 o4 o�3 X{�$ oh 
2�* 0 O  o s�3 
oh 
0oh 
-*oh 
Y8  (G4 	o4 ;�   	o4 ;�   	o4 
@�   	o�3 ,o4 9�   	o�3 ,o4 9�   	o*4 8�   o4 	3ws�3 o4 o�2 o�3 o4 o�2 o�3 o4 o�2 o�3 o4 o�3 o4 o�2 o�3 o 
o�3 .o*4 :n���+Y<����oh 
�_,o� 
* 0 p   p oh 
Y
+`(G4 o4 
3Jo�3 ,o4 ,6+.	o4 	.
	o4 
3	o4 {�$ 	oc4 	o*4 	-�*Y
/�*0 �   q {�$ oh 
Y
8�   {�$ oG4 o4 9�   o4 3*o4 .o4 .
o4 3foh 
Y+W(G4 	3*	o4 3<	o4 o�3 3-	+o4 {�$ oc4 o*4 ,3�*Y/�Y
<N���*  0 n   f {�$ ,d{�$ ob4 {�$ oh 
Y
+E{�$ oG4 o$4 0+o$4 1*o4 ,o4 3o$4 *o4 .Y
/�*Foh 
Y(]4 *&(^4 *  0 G   f {�$ ,={�$ oh 
Y
+){�$ oG4 o4 3o4 -o$4 *Y
/�* 0 m   r 
+(G4 o%4 X
oh 
2�
+A(G4 Y(G4 o"4 -o*4 +o4 -
o*4 -�o+4 X
oh 
2�*   0 F   m oh 
Y
+(G4 o$4 2o4 -Y
/�oh 
XY1
Xo� 
*  0 6   	  (b4 oh 

+o$4 Y(G4 o$4 0Y
0�o 
*  0 h       o 
{�$ ,Wo%4 o'4 X+(G4 o%4 Xoh 
2�o4 ,{�$ -sF4 }�$ {�$ oc4 *0 Q       (d4 (P4 ,o*4 3r��p(� 
o+4 +%o"4 Xo#4 o)4 -�(R4 *   0 �   s (G4 
{�$ ,&X+(G4 	o%4 	o'4 X2�o� 
{�$ ,So*4 +"o4 -o"4 Yo#4 o*4 -�+(G4 o%4 Xoh 
2�(R4 *joh 
0*oh 
Y(G4 *"}�$ *  0 �   t o$4 
jo"4 XX+A	(G4 o4 3*j3
o84 +o84 /o4 ,o84 	X	1�o94 +*(G4 o4 3o84 Yo94 X1�*  0 �   u o$4 
o"4 XX+F(G4 o4 3,o4 ,#-o4 o�2 +o4 o�2 .X-	1�-@,=+3(G4 o4 	.o4 
3o4 o�2 X	1�*   0   v 
o$4 o4 oE4 	>�   s�3 s�3 o4 s�2 o 4 o4 	(G4 o4 o�2 o�2 	o$4 YYo"4 Yo#4 X(G4 Yo#4 	(d4 	(P4 	(d4 	X(P4 o+4 o)4 o+4 o)4 +o"4 Xo#4 o)4 -�
o4 o44 +o44 *   0 *  w o4 
o4 o�2 o�3 -+o34 oh 
oz3 2oz3 +oh 
8�   oG4 o�3 o[3 	on3 ,Rol3 ,CX+3o�3 on3 ,Roj3 ,oG4 o24 X2�+X	,P
+&	oD4 Xo=4 	.o=4 	0
	oh 
2�
Yo14 1
Yo24 ?)���*  0 �  x 
8�  oG4 o34 9v  oh 
9k  oh 
o4 sF4 +og 
&X	2�8&  oG4 o4 o�2 	o4 
		oz3 /		oz3 
oh 
/	
oh 
8�   
oG4 	o�3 on3 ,5oG4 ,o/4 Xo04 o14 Xo24 +Zop3 ,o04 o 
+
o 
X+o 
Xo14 X2�o14 XX/oh 
?8���Xoh 
?����X
oh 
?e���* 0 �       (� 
sU2 }�$ sF4 }�$ {�$ oh4 s�3 }�$ s�3 }�$ s�3 }�$ s�3 }�$ j}�$ j}�$ j}�$ }�$ }�$ }�$ *6{�$ oY2 *{�$ *{�$ *{�$ *{�$ *{�$ *{�$ *2{�$ oX2 *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *{�$ *"}�$ *�(� 
(� 
o� 
}�$ }�$ (�4 *�{�$ sh2 }�$ sn4 }�$ {�$ op4 oV2 sy 
}�$ * 0 |  y 
s[2 {�$ op4 oh 
8T  {�$ {�$ ov4 oi2 
:<  o]2 E   )   6   �   �   �   �   �   k   d   �         8�   {�$ op4 oV2 8�   (�4 8�   
8�   (�4 {�$ op4 oh 
1{�$ op4 oh 
	0�(�4 *8�   o_2 ,-oT2 _,,;,#oR2 3{�$ ov4 o�2 3+{�$ ov4 o�2 ,.(�4 +#(�4 +(�4 +{�$ ov4 (�4 9����*2{�$ o� 
*{�$ *"}�$ *{�$ *"}�$ * 0 F   z {�$ ou4 
++oG4 o4 .o4 	.
o4 3*Xoh 
2�*  0 3  { {�$ ou4 
+.oh 
YoG4 o	4 , o4 ,oh 
Yof4 oh 
0�(�4 -(�4 ,:oh 
11oh 
YoG4 	o	4 ,{�$ ov4 ,	(�4 oT4 {�$ oO4 (�4 -(�4 +,{�$ r2�po� 
&+{�$ r��po� 
&+#oG4 {�$ o,4 o� 
&Xoh 
2�,{�$ r��po� 
&*{�$ r�po� 
&* 0 �  | {�$ ou4 
oh 
Y8�   oG4 o4 36o4 o�2 YE            +P+K+E+@o4 35o4 o�2 YE            ++	+Y29d���,-*	9�   9�   o$4 	o$4 X>�   ~� 
	o$4 X+*oG4 o4 3o,4 (" 
Xo$4 2�o� 
1o� 
 3
o� 
r��po$ 
-(�4 +4r��po$ 
-(�4 +r��po$ 
-
(�4 	,$, o$4 	o$4 YX	o$4 of4 ,o$4 of4 ,o$4 of4 o$4 	o$4 o$4 YY
o)4 o$4 of4 o$4 of4 9�   
9�   	oG4 o4 3uo,4 r�po$ 
o,4 r�)pXo� 
o,4 o� 
r$�pr�)p(� 
o,4 Xo� 
(" 
o-4 9~  o	4 ,+	+oG4 o
4 ,X,		
X2�,$o4 .
92  	
oe4 8  o4 @  {�$ o�3 	+[oG4 o4 3Ao4 -8o,4 s� 
o,4 o� 
&r<�po� 
&o� 
o-4 X	
X2�	
XY+oG4 o	4 ,XY	/�1cX(�4 XoG4   o4 3A o4 -8o,4 s� 
!! o,4 o� 
&!r<�po� 
& !o� 
o-4 	"+"oG4 ##o
4 "X"-
"oh 
2�,s$
	Y%+N%oG4 &&o	4 -	&o4 ,
X
&o4 -$X$&o
4 ,
&o"4 Y
+%Y%%/�$1	
Y
(�4 *�02
900Y*a2f0
XaY*A2F0
XAY**F02
900Y**   0 L   } 
rV�po$ 
.7o� 
YYo� 

r�)po$ 
.	o� 

(� 
-
*0 �  ~ s�3 
{�$ oo4 s�2 o 4 
8�   o� 
 3X8�   o� 
"3nXo� 
<�   	+	o� 
".	X		o� 
2�	Yo� 

,
+,
+-
	X+Po� 
\3>Xo� 
/4o� 
l.t.
++X+Xo� 
?���sy 
,	o� 
&	,r�*po� 
&	o� 
&+<o_ 
\3)Xo] 
/Xo_ 
\3Xo� 
&Xo] 
2�o� 
o4 o4 o� 
0**0 q   s�3 
{�$ oo4 s�2 o 4 8  	o� 
 3		X8�   @�   	o� 
0?�   	o� 
9=�   	o� 
03c	o� 
Y/X	Xo� 
x3K	X+ ��  /Z	o� 
(�4 X	X	o� 
<�   	o� 
 .{	o� 
\3�+n+ ��  /
Z	o� 
(�4 X	X	o� 
/B	o� 
 .7	o� 
\3�+*	o� 
\3	X	o� 
/(�4 +	X	o� 
?����3*(�4 o,4 ,o,4 o� 
0**   0 �  � 
JXTo� 
j5#a.)fYE   �   O  O  O     *s.u.*T*T*T*Jo� 
/Jo� 
 3JXTJ
+JXTJo� 
/Jo� 
 3�JYo� 
#        	( 
([ 
�&�&� 9�   o4 #       @Z#      �?Xjo�2 *Jo� 
/Jo� 
 3JXTJo� 
/Jo� 
"3JXTJ
+JXTJo� 
/Jo� 
"3�JYo� 
JXT,(o� 
1o4 {�$ oq4 o�3 jo�2 *       � � �   � �  0 1  � o3 	YE   
            +r@yp
+r*yp
+~� 

o3 #        2o3 #        �+{�$ 9�  ~� 
(
 
9�  :�  {�$ {�$ o�> 
o3 	.{�$ o3 om2 �Js� 
{�$ o3 om2 joM	 
(� 
�,o.  
�
,
o.  
�{�$ X}�$ o3 s�3 o 4 sy 
rb�po� 
&rr�po� 
&o3 #        .o3 o3 #      Y@[Z+o3 ( 
(� 
o� 
&r�)po� 
&r��po� 
&o3 o3 #      Y@[Zo3 #        .o3 o3 #      Y@[Z+o3 ( 
(� 
o� 
&r�)po� 
&o3 ,9o
3 Yr��po� 
&( 
(� 
o� 
&r�)po� 
&r��po� 
&r�)po� 
&rd�po� 
&r��po� 
&r�po� 
&r$�po� 
&o� 
&r(�po� 
&r.�po� 
&rZ�po� 
&r`�po� 
&r��po� 
&o� 
o-4 {�$ ou4 		oH4 		oh 
YoQ4 *{�$ ol2 *      � '�      � ]�     0 �   � ..&+~ ��  <�   �� 
ѝs� 
o�3 * ��  /l �  (z 
    0+�a 3	Ҝ+	c �   _Ҝ	 �   _Ҝ	o{ 
o�3 *    /�s� 
o�3 * 0 �   � {�$ ou4 
o_4 ?�   oG4 oQ4 o4 ,+5o"4 XX+oG4 o4 -X	,1�{�$ oM4 	,{�$ o�4 ~� 
o-4 {�$ o�4 *  0 8   � {�$ ou4 
o_4 2{�$ ov4 ,(�4 oQ4 *0 b  � {�$ ou4 
o�2 YE   �   �         �       �        K   K   K       �   *o`4 ?�   s�3 s�2 o 4 o4 oH4 oG4 o4 (�4 *o`4 ?�   s�3 s�2 o 4 o4 oH4 oG4 o4 *(�4 *(�4 *{�$ os4 o�3 ,Oo�3 o�3 s�2 o�3 *{�$ oq4 o�3 	,	o�3 	o�3 *{�$ oq4 o�3 *  0   � {�$ op4 oh 
>�   {�$ ou4 
{�$ oo4 {�$ oo4 o�2 o�2 .	(�4 +$o�2 3{�$ oq4 o�3 	,	o�3 {�$ op4 oW2 o�2 3{�$ {�$ ow4 on2 +{�$ o�2 on2 o�2 3!{�$ oy4 j2{�$ oy4 (�4 *o�2 j/ {�$ oy4 j2{�$ oy4 (�4 *  0 l   � {�$ ov4 
-*o�2 {�$ oq4 o�2 io�3 ,9o�3 3{�$ ow4 o�2 +o�3 o�2 {�$ o�2 on2 *0 +  � {�$ ov4 
-*oR2  �  =J   �  =9     =�   �   =N  8YEi   �  :  :  :  :  