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

// https://en.wikipedia.org/wiki/BlinnñPhong_shading_model
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

// Helper function for iterating over a sparse list of bits.  Gets4 ,*(4 ,*(4 3(4 *(4 .	(4 3( 4 *(,4 sâ 

rn‡poä 
&(Î3 oä 
&rd‡poä 
&(
4 ,rx„poä 
&oÌ 
(-4 *  0 ‹   _ sy 

(4 (!4 r~„poä 
&(¯3 oΩ2 oΩ2 .Ror4 oΩ2 io¡3 	,<	o∏3 -4r4·poä 
&	o∂3 ˛U  oÌ 
oä 
&r·)poä 
&(Ú3 o±2 o±2 .rä„poä 
&oØ2 oØ2 .r–„poä 
&rd‡poä 
&oÌ 
(-4 *0 %   1  (,4 sâ 

r‰poä 
&oÌ 
(-4 *   0    1  sy 

r"‰poä 
&oÌ 
(-4 * 0 %   1  (,4 sâ 

rJ‰poä 
&oÌ 
(-4 *   0 -   K ($4 
($4 ("4 Xo$4 1o$4 ˛˛**   0 ^   ` (&4 
(4 .*(Ï3 -*o$4 Xo$4 o"4 X+	oG4 3*o
4 ,*	Y	1›*  0 b   a (&4 
sF4 (4 3I($4 X($4 ("4 X+,oG4 o4 3oÏ3 3oH4 X	1–*  0 b   a (&4 
sF4 (4 3I($4 X($4 ("4 X+,oG4 o4 3oÏ3 3oH4 X	1–*  0 m   b (&4 
(4 3X(Ï3 ,Lo$4 Xo$4 o"4 X+-	oG4 .$o4 3oÏ3 3X	X	1Œ*   0 l  c (&4 
(4 sC4 8C  	oG4 o4 o˝2 j8  oÜ3 	j
oj3 :Í   8î   oj 
tÃ o=4 o[3 3oA4 -
3oB4 	+fo=4 o[3 1:s<4 o@4 o[3 o>4 
˛oB4 o 
	+o=4 
Xoh 
?_ˇˇˇ	-4s<4 o@4 o[3 o>4 
˛oB4 og 
&o[3 Xoz3 ?‰˛ˇˇ	X	oh 
?±˛ˇˇ*Í{ø$ ./{ø$ .&{ø$ .{ø$ .{ø$ .
{ø$ ˛** 0 f       {ø$ .[{ø$ 	.Q{ø$ 
.G{ø$ .={ø$ .3{ø$ .){ø$ .{ø$ .{ø$ .{ø$ ˛***{ø$ ˛*Æ{ø$ .{ø$ .{ø$ .{ø$ ˛**   0 +   2  (4 ,*{ø$ 3(,4 oı 

oˆ 
˛**J($4 2{Ω$ **"}Ω$ *{æ$ *"}æ$ *N(4 3{Õ$ **"}Õ$ *÷($4 /*(4 3*(4 ,
(4 -*(4 -**{Ã$ *"}Ã$ *V("4 0
(,4 ˛**{ $ *"} $ *{À$ *"}À$ *{ø$ *{¿$ *"}¿$ *   0 3   S (*4 
,o4 3o*4 
(4 .-(ú2 *o4 *{√$ *22}√$ *{ƒ$ *"}ƒ$ *{∆$ *"}∆$ *:($4 ("4 X*{≈$ *í{≈$ -(&4 ,(&4 oW4 *{≈$ *"}≈$ *{¡$ *"}¡$ *{¬$ *{»$ *"}»$ *{…$ *"}…$ *{«$ *"}«$ * 0 ì   d (	4 ,(4 oø2 *(4 3%(4 oˇ2 ,(4 o˝2 oÉ3 *(64 *(4 	.	(4 3(4 o„2 *(*4 
+#o4 .
	.3o54 *o*4 
-⁄*Z(*4 ,(*4 o54 **J(54 (64 ˛˛*Ü(64 .(4 o’2 *(4 o◊2 *é(64 3(4 oÿ2 *(4 o÷2 *Ü(64 .(4 o◊2 *(4 o’2 *   0 >      çá %r£  p¢%r£  p¢%rt‰p¢%r~‰p¢%rÑ‰p¢%rà‰p¢%rl◊p¢%rû∂p¢%rå‰p¢%	rò‰p¢%
r§‰p¢%r™‰p¢Ä∫$ çç %–M; (\ 
Äª$ çá %r£  p¢%r£  p¢%rQ– p¢%rO≥ p¢%rÁ© p¢%rsæ p¢%rﬂ´ p¢%ryã p¢%r«™ p¢%	rè∂ p¢%
rÁ∂ p¢%rY÷ p¢%r£÷ p¢%rë÷ p¢%re÷ p¢%rÕ p¢%r7° p¢%rÁ° p¢%r∞‰p¢%rº‰p¢Äº$ *v(È 
j}Œ$ }œ$ }–$ *{Œ$ *"}Œ$ *{œ$ *"}œ$ *{–$ *"}–$ *6oj 
tÃ *   0 Z   e 
+I(D4 oA4 -52o?4 o$4 1#o?4 o4 o˝2 oÖ3 -o?4 o$4 
Xoh 
2Æ*^d(q 
}—$ }“$ *6oj 
tÀ *:oh 
(d4 * 0 !   S (g4 
oh 
1oh 
Y(f4 *   0 '   f oh 
Y
+(G4 o4 ,*Y
/Ê*äoh 
1oh 
Y(G4 o4 ˛**  0 ©  g sF4 
(G4 o"4 	Xoh 
2oh 
YY	X,X8S  (G4 o	4 9’   o74 9…   o)4 9Ω   X+M(G4 		o	4 ,@	o4 .6	o4 oø2 o4 oø2 3	o)4 o)4 3X1≠Y>µ   sÈ3 

o*4 o4 só2 o 4 
o4 o4 oø2 o¿2 o)4 
(e4 X+bo4 
3
(i4 +Mo4 	3
(j4 +8o4 3og 
&(k4 X+o4 3
(l4 X>§˛ˇˇ,(m4 *   0   h oh 
//*(L4 (G4 
o"4 Xoh 
2oh 
YYX8Ω   	(G4 o"4 -oÔ3 8Ö   oı3 o,4 sâ 
o"4 	X	X+(G4 		o,4 oä 
&X1€oÌ 
o-4 o˛3 o4 	X(f4 Y(R4 o14 -~ã 
o-4 	Y	<<ˇˇˇ*0 
  h oh 
//*(L4 (G4 
o"4 Xoh 
2oh 
YYX8ª   	(G4 o"4 -	.oÔ3 8î   o"4 >á   	.oı3 o,4 sâ 
o"4 	X	X+(G4 		o,4 oä 
&X1€oÌ 
o-4 	.o˛3 o4 	X(f4 Y	Y	<>ˇˇˇ*  0    	  
+(M4 X
oh 
2Î*  0 f   i oh 
/2Xoh 
2*(G4 
o4 -*o4 o#4 Xo"4 X+(G4 	o+4 	o"4 XX1‚*  0 s   j oh 
//*(G4 
o4 -*(R4 (S4 oh 
Y+(G4 o4 ,(Q4 Y0·oh 
YY(P4 (R4 (S4 * 0 G   k (ê 
,?
+2(G4 X+Xo(4 1Ûo*4 +	o*4 	-ˆX
oh 
2≈* 0 L   l (ê 
,D
+7(G4 o*4 o4 	
YE                       X
oh 
2¿*0 (   	  
+(G4 o4 ,(Q4 *X
oh 
2›*0 ]   m 
{“$ ,Q{“$ ob4 {“$ oh 
Y+2{“$ oG4 o4 ,o4 3X
+
o4 .Y/ *&(U4 * 0 R   f {“$ ,H{“$ ob4 {“$ oh 
Y
+){“$ oG4 o4 ,o$4 o$4 /*Y
/”*  0 }   f {“$ ,s{“$ ob4 {“$ oh 
Y
+T{“$ oG4 o4 ,;o4 .o4 .o4 .
o4 3o4 *o4 3*Y
/®*   0 ”   n só3 
{“$ 9¿   {“$ ob4 +'{“$ oG4 	o4 ,	o4 3XX{“$ oh 
2À+f{“$ oG4 o4 ,Ho4 	3=o4 oÎ2 o4 o·2 o4 oÔ2 o4 oÒ2 o4 oô3 X{“$ oh 
2ã* 0 O  o só3 
oh 
0oh 
-*oh 
Y8  (G4 	o4 ;˙   	o4 ;Ì   	o4 
@’   	oÏ3 ,o4 9∆   	oÏ3 ,o4 9°   	o*4 8ã   o4 	3wsã3 o4 oÎ2 oç3 o4 oÔ2 oè3 o4 oÒ2 oë3 o4 oì3 o4 o·2 oñ3 o 
oå3 .o*4 :nˇˇˇ+Y<Ò˛ˇˇoh 
˛_,oÚ 
* 0 p   p oh 
Y
+`(G4 o4 
3JoÏ3 ,o4 ,6+.	o4 	.
	o4 
3	o4 {“$ 	oc4 	o*4 	-œ*Y
/ú*0 ∆   q {“$ oh 
Y
8´   {“$ oG4 o4 9è   o4 3*o4 .o4 .
o4 3foh 
Y+W(G4 	3*	o4 3<	o4 oÏ3 3-	+o4 {“$ oc4 o*4 ,3Ÿ*Y/•Y
<Nˇˇˇ*  0 n   f {“$ ,d{“$ ob4 {“$ oh 
Y
+E{“$ oG4 o$4 0+o$4 1*o4 ,o4 3o$4 *o4 .Y
/∑*Foh 
Y(]4 *&(^4 *  0 G   f {“$ ,={“$ oh 
Y
+){“$ oG4 o4 3o4 -o$4 *Y
/”* 0 m   r 
+(G4 o%4 X
oh 
2Ê
+A(G4 Y(G4 o"4 -o*4 +o4 -
o*4 -Ìo+4 X
oh 
2∂*   0 F   m oh 
Y
+(G4 o$4 2o4 -Y
/ﬂoh 
XY1
XoÚ 
*  0 6   	  (b4 oh 

+o$4 Y(G4 o$4 0Y
0‚o 
*  0 h       o 
{—$ ,Wo%4 o'4 X+(G4 o%4 Xoh 
2Âo4 ,{“$ -sF4 }“$ {“$ oc4 *0 Q       (d4 (P4 ,o*4 3rŒ‰p(œ 
o+4 +%o"4 Xo#4 o)4 -Á(R4 *   0 ö   s (G4 
{—$ ,&X+(G4 	o%4 	o'4 X2‚oÚ 
{—$ ,So*4 +"o4 -o"4 Yo#4 o*4 -⁄+(G4 o%4 Xoh 
2Â(R4 *joh 
0*oh 
Y(G4 *"}—$ *  0 ö   t o$4 
jo"4 XX+A	(G4 o4 3*j3
o84 +o84 /o4 ,o84 	X	1ªo94 +*(G4 o4 3o84 Yo94 X1—*  0 ≠   u o$4 
o"4 XX+F(G4 o4 3,o4 ,#-o4 o„2 +o4 o„2 .X-	1≤-@,=+3(G4 o4 	.o4 
3o4 o‰2 X	1»*   0   v 
o$4 o4 oE4 	>Ê   sÈ3 sÈ3 o4 só2 o 4 o4 	(G4 o4 o˝2 o˛2 	o$4 YYo"4 Yo#4 X(G4 Yo#4 	(d4 	(P4 	(d4 	X(P4 o+4 o)4 o+4 o)4 +o"4 Xo#4 o)4 -„
o4 o44 +o44 *   0 *  w o4 
o4 o˝2 oÏ3 -+o34 oh 
oz3 2oz3 +oh 
8Œ   oG4 oÜ3 o[3 	on3 ,Rol3 ,CX+3oÜ3 on3 ,Roj3 ,oG4 o24 X2«+X	,P
+&	oD4 Xo=4 	.o=4 	0
	oh 
2–
Yo14 1
Yo24 ?)ˇˇˇ*  0 £  x 
8è  oG4 o34 9v  oh 
9k  oh 
o4 sF4 +og 
&X	2Ï8&  oG4 o4 o˝2 	o4 
		oz3 /		oz3 
oh 
/	
oh 
8¥   
oG4 	oÜ3 on3 ,5oG4 ,o/4 Xo04 o14 Xo24 +Zop3 ,o04 o 
+
o 
X+o 
Xo14 X2‚o14 XX/oh 
?8ˇˇˇXoh 
?Ã˛ˇˇX
oh 
?e˛ˇˇ* 0 Ç       (È 
sU2 }”$ sF4 }‘$ {‘$ oh4 s¨3 }’$ sΩ3 }÷$ s÷3 }◊$ s‰3 }ÿ$ j}Ÿ$ j}⁄$ j}€$ }›$ }ﬁ$ }ﬂ$ *6{”$ oY2 *{”$ *{’$ *{÷$ *{◊$ *{ÿ$ *{‘$ *2{”$ oX2 *{‹$ *"}‹$ *{Ÿ$ *"}Ÿ$ *{⁄$ *"}⁄$ *{€$ *"}€$ *{›$ *"}›$ *{ﬁ$ *"}ﬁ$ *{ﬂ$ *"}ﬂ$ *ñ(È 
(≤ 
o≥ 
}‡$ }‰$ (Ü4 *‚{‡$ sh2 }‚$ sn4 }„$ {„$ op4 oV2 sy 
}·$ * 0 |  y 
s[2 {„$ op4 oh 
8T  {‚$ {„$ ov4 oi2 
:<  o]2 E   )   6   ‘   ›   Ê   ˜   ˜   k   d   ˜         8Ú   {„$ op4 oV2 8€   (õ4 8Œ   
8«   (õ4 {„$ op4 oh 
1{„$ op4 oh 
	0‘(é4 *8å   o_2 ,-oT2 _,,;,#oR2 3{„$ ov4 oû2 3+{„$ ov4 oü2 ,.(ù4 +#(û4 +(ü4 +{„$ ov4 (ñ4 9¶˛ˇˇ*2{·$ oÌ 
*{‰$ *"}‰$ *{„$ *"}Â$ * 0 F   z {„$ ou4 
++oG4 o4 .o4 	.
o4 3*Xoh 
2Ã*  0 3  { {„$ ou4 
+.oh 
YoG4 o	4 , o4 ,oh 
Yof4 oh 
0…(â4 -(ç4 ,:oh 
11oh 
YoG4 	o	4 ,{„$ ov4 ,	(°4 oT4 {„$ oO4 (â4 -(ç4 +,{·$ r2Âpoä 
&+{·$ r˚Âpoä 
&+#oG4 {·$ o,4 oä 
&Xoh 
2”,{·$ r∏Êpoä 
&*{·$ r‰poä 
&* 0 ı  | {„$ ou4 
oh 
Y8ë   oG4 o4 36o4 oû2 YE            +P+K+E+@o4 35o4 oû2 YE            ++	+Y29dˇˇˇ,-*	9“   9À   o$4 	o$4 X>∑   ~ã 
	o$4 X+*oG4 o4 3o,4 (" 
Xo$4 2Àoˆ 
1o∆ 
 3
oç 
rŒÊpo$ 
-(ì4 +4r‚Êpo$ 
-(î4 +rÊpo$ 
-
(í4 	,$, o$4 	o$4 YX	o$4 of4 ,o$4 of4 ,o$4 of4 o$4 	o$4 o$4 YY
o)4 o$4 of4 o$4 of4 9ê   
9â   	oG4 o4 3uo,4 rÁpo$ 
o,4 r·)pXoë 
o,4 o— 
r$Ápr·)p(à 
o,4 Xoç 
(" 
o-4 9~  o	4 ,+	+oG4 o
4 ,X,		
X2€,$o4 .
92  	
oe4 8  o4 @  {„$ oı3 	+[oG4 o4 3Ao4 -8o,4 sâ 
o,4 oä 
&r<Ápoä 
&oÌ 
o-4 X	
X2ú	
XY+oG4 o	4 ,XY	/€1cX(•4 XoG4   o4 3A o4 -8o,4 sâ 
!! o,4 oä 
&!r<Ápoä 
& !oÌ 
o-4 	"+"oG4 ##o
4 "X"-
"oh 
2Ÿ,s$
	Y%+N%oG4 &&o	4 -	&o4 ,
X
&o4 -$X$&o
4 ,
&o"4 Y
+%Y%%/≠$1	
Y
(•4 *÷02
900Y*a2f0
XaY*A2F0
XAY**F02
900Y**   0 L   } 
rVÁpo$ 
.7oˆ 
YYo— 

r·)po$ 
.	o— 

(í 
-
*0 ∏  ~ sÈ3 
{„$ oo4 só2 o 4 
8·   o∆ 
 3X8    o∆ 
"3nXoˆ 
<´   	+	o∆ 
".	X		oˆ 
2‰	Yo— 

,
+,
+-
	X+Po∆ 
\3>Xoˆ 
/4o∆ 
l.t.
++X+Xoˆ 
?ˇˇˇsy 
,	oä 
&	,r´*poä 
&	oä 
&+<o_ 
\3)Xo] 
/Xo_ 
\3Xoì 
&Xo] 
2πoÌ 
o4 o4 oˆ 
0**0 q   sÈ3 
{„$ oo4 só2 o 4 8  	o∆ 
 3		X8ˇ   @Œ   	o∆ 
0?¿   	o∆ 
9=≤   	o∆ 
03c	oˆ 
Y/X	Xo∆ 
x3K	X+ ˇˇ  /Z	o∆ 
(ê4 X	X	oˆ 
<Ü   	o∆ 
 .{	o∆ 
\3ø+n+ ˇˇ  /
Z	o∆ 
(ë4 X	X	oˆ 
/B	o∆ 
 .7	o∆ 
\3¬+*	o∆ 
\3	X	oˆ 
/(ï4 +	X	oˆ 
?·˛ˇˇ3*(ó4 o,4 ,o,4 oˆ 
0**   0 â  Ä 
JXTo∆ 
j5#a.)fYE   ∑   O  O  O     *s.u.*T*T*T*Joˆ 
/Jo∆ 
 3JXTJ
+JXTJoˆ 
/Jo∆ 
 3‰JYo— 
#        	( 
([ 
ﬁ&ﬁ&ﬁ 9ª   o4 #       @Z#      ‡?Xjo∂2 *Joˆ 
/Jo∆ 
 3JXTJoˆ 
/Jo∆ 
"3JXTJ
+JXTJoˆ 
/Jo∆ 
"3‰JYo— 
JXT,(oˆ 
1o4 {„$ oq4 o±3 jo∏2 *       ´ ∫ ‹   ´ ¿  0 1  Å o3 	YE   
            +r@yp
+r*yp
+~ã 

o3 #        2o3 #        ˛+{Â$ 9Ω  ~ã 
(
 
9≠  :ß  {Â$ {Ê$ oÊ> 
o3 	.{‚$ o3 om2 ﬁJsî 
{‚$ o3 om2 joM	 
(ï 
ﬁ,o.  
‹
,
o.  
‹{Ê$ X}Ê$ o3 sÈ3 o 4 sy 
rbÁpoä 
&rrÁpoä 
&o3 #        .o3 o3 #      Y@[Z+o3 ( 
(ó 
oä 
&r·)poä 
&rÑÁpoä 
&o3 o3 #      Y@[Zo3 #        .o3 o3 #      Y@[Z+o3 ( 
(ó 
oä 
&r·)poä 
&o3 ,9o
3 YròÁpoä 
&( 
(ó 
oä 
&r·)poä 
&r–Ápoä 
&r·)poä 
&rd‡poä 
&rÓÁpoä 
&rËpoä 
&r$Ápoä 
&oä 
&r(Ëpoä 
&r.Ëpoä 
&rZËpoä 
&r`Ëpoä 
&rÄËpoä 
&oÌ 
o-4 {„$ ou4 		oH4 		oh 
YoQ4 *{‚$ ol2 *      ¬ 'È      ò ]ı     0 £   Ç ..&+~ ˇˇ  <ç   çì 
—ùs∞ 
oÙ3 * ˇˇ  /l §  (z 
    0+ça 3	“ú+	c ˇ   _“ú	 ˇ   _“ú	o{ 
oÙ3 *    /—sñ 
oÙ3 * 0 ¢   É {„$ ou4 
o_4 ?Ö   oG4 oQ4 o4 ,+5o"4 XX+oG4 o4 -X	,1‹{„$ oM4 	,{„$ oÄ4 ~ã 
o-4 {„$ oÇ4 *  0 8   Ñ {„$ ou4 
o_4 2{„$ ov4 ,(¢4 oQ4 *0 b  Ö {„$ ou4 
oû2 YE   Ú   œ         ù       ñ        K   K   K       è   *o`4 ?Ò   sÈ3 só2 o 4 o4 oH4 oG4 o4 (è4 *o`4 ?ß   sÈ3 só2 o 4 o4 oH4 oG4 o4 *(ô4 *(ò4 *{„$ os4 o⁄3 ,Oo’3 oœ3 só2 oÀ3 *{„$ oq4 o≥3 	,	o†3 	o¢3 *{„$ oq4 o≤3 *  0   Ü {„$ op4 oh 
>˜   {„$ ou4 
{„$ oo4 {„$ oo4 oû2 oû2 .	(ö4 +$oû2 3{„$ oq4 o≥3 	,	o¢3 {„$ op4 oW2 oπ2 3{‚$ {„$ ow4 on2 +{‚$ oπ2 on2 oû2 3!{„$ oy4 j2{„$ oy4 (ú4 *o∑2 j/ {„$ oy4 j2{„$ oy4 (ú4 *  0 l   á {„$ ov4 
-*o∏2 {„$ oq4 o∑2 ioÆ3 ,9o£3 3{„$ ow4 o∫2 +o£3 o∫2 {‚$ oπ2 on2 *0 +  à {„$ ov4 
-*oR2  À  =J   ∫  =9     =†   Ï   =N  8YEi   ›  :  :  :  :  