//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using System.IO;

namespace UnrealBuildTool.Rules
{
	public class SpatialMappingEditor : ModuleRules
	{
        public SpatialMappingEditor(ReadOnlyTargetRules Target) : base(Target)
        {
            PrivatePCHHeaderFile = "SpatialMappingEditor/Public/SpatialMappingEditor.h";

            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
            PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

            PrivateDependencyModuleNames.AddRange(new string[]
                {"Slate",
                    "SlateCore" });

            PublicDependencyModuleNames.AddRange(
                new string[]
                {
                    "Stereolabs",
                     "SpatialMapping"
                }
                );

            PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
                    "Slate",
                    "SlateCore",
                    "Engine",
                    "UnrealEd",
                    "HeadMountedDisplay",
                    "DesktopPlatform",
                    "InputCore"
                }
				);
		}
	}
}