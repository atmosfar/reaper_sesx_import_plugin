#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <stdarg.h>
#include <algorithm>

#ifdef _WIN32
  #define strcasecmp _stricmp
#endif

#include "pugixml.hpp"
#include "reaper_plugin.h"
#include "reaper_plugin_functions.h"

// PCM_source and related types (simplified from the full SDK to avoid WDL dependency)
class IReaperControlSurface;
class PCM_source
{
public:
  virtual ~PCM_source() {}
  virtual PCM_source *Duplicate()=0;
  virtual const char *GetType()=0;
  virtual const char *GetFileName()=0;
  virtual double GetLength()=0;
  virtual int GetNumChannels()=0;
  virtual double GetSampleRate()=0;
  virtual double GetLengthSeconds()=0;
};

// Define the global function pointers
void (*ShowConsoleMsg)(const char* msg) = nullptr;
int (*ShowMessageBox)(const char* msg, const char* title, int type) = nullptr;
PCM_source* (*PCM_Source_CreateFromFile)(const char* filename) = nullptr;

// Helper Structs
struct FileInfo {
    std::string absolutePath;
    std::string relativePath;
    std::string format;
};

struct Keyframe {
    long long sampleOffset;
    double value;
    std::string type;
};

struct ClipInfo {
    std::string name;
    long long startPoint;
    long long endPoint;
    long long sourceInPoint;
    std::string fileID;
    double fadeInDuration;
    int fadeInShape;
    double fadeOutDuration;
    int fadeOutShape;
    float volume;
    float pan;
    std::vector<Keyframe> volumeKeyframes;
    std::vector<Keyframe> panKeyframes;
};

struct MarkerInfo {
    std::string name;
    long long startTime;
    long long duration;
    int index;
};

struct TrackInfo {
    std::string name;
    float hue;
    int peakcol;
    bool mute;
    bool solo;
    bool armed;
    bool monitoring;
    int automationMode;
    float volume;
    float pan;
    std::vector<ClipInfo> clips;
    std::vector<Keyframe> volumeKeyframes;
    std::vector<Keyframe> panKeyframes;
    std::vector<Keyframe> muteKeyframes;
};

// --- Helper Functions ---

// Convert HSL to RGB (mirroring Python's colorsys.hsv_to_rgb logic)
// SESX uses Hue (0-360), Saturation (0.5), Value (1.0)
void HSVtoRGB(float h, float s, float v, int& r, int& g, int& b) {
    float f, p, q, t;
    int i;

    if (s == 0) {
        r = g = b = (int)(v * 255);
        return;
    }

    h /= 60.0f;
    i = (int)floor(h);
    f = h - i;
    p = v * (1.0f - s);
    q = v * (1.0f - (s * f));
    t = v * (1.0f - (s * (1.0f - f)));

    float rf, gf, bf;
    switch (i % 6) {
        case 0: rf = v; gf = t; bf = p; break;
        case 1: rf = q; gf = v; bf = p; break;
        case 2: rf = p; gf = v; bf = t; break;
        case 3: rf = p; gf = q; bf = v; break;
        case 4: rf = t; gf = p; bf = v; break;
        case 5: rf = v; gf = p; bf = q; break;
        default: rf = 0; gf = 0; bf = 0; break;
    }

    r = (int)(rf * 255);
    g = (int)(gf * 255);
    b = (int)(bf * 255);
}

int hue_to_peakcol(float track_hue) {
    float hue = fmod(track_hue, 360.0f);
    float saturation = 0.5f;
    float value = 1.0f;

    int r, g, b;
    HSVtoRGB(hue, saturation, value, r, g, b);

    // Create PEAKCOL integer: (0x1 << 24) | (blue << 16) | (green << 8) | red
    return (0x1 << 24) | (b << 16) | (g << 8) | r;
}

int fade_type_to_shape(const std::string& type) {
    if (type == "lin") return 0; // Linear
    if (type == "log") return 1; // Logarithmic
    return 0; // Default
}

double convert_sesx_volume_to_volenv(double v, double& original_dB) {
    // Polynomial coefficients for v^10 down to v^0
    static const double coeffs[] = {
        -2.09233708e+04, 1.21079132e+05, -3.04166739e+05, 4.35852234e+05,
        -3.94242081e+05, 2.35794282e+05, -9.52128435e+04, 2.61473496e+04,
        -4.91584283e+03, 6.76296702e+02, -7.34158940e+01
    };

    original_dB = 0.0;
    for (int i = 0; i <= 10; ++i) {
        original_dB += coeffs[i] * pow(v, 10 - i);
    }

    // Clamp to Reaper's operational range and convert to linear scale
    double dB_clamped = original_dB;
    if (dB_clamped > 6.0) dB_clamped = 6.0;
    if (dB_clamped < -60.0) dB_clamped = -60.0;

    double volenv = pow(10.0, dB_clamped / 20.0);

    // Ensure we don't exceed Reaper's maximum VOLENV value of 2.0
    return (volenv > 2.0) ? 2.0 : volenv;
}

double convert_sesx_pan_to_reaper(double v) {
    // Audition Pan: 0 (100% Left) to 1 (100% Right)
    // Reaper Pan: -1 (100% Left) to 1 (100% Right)
    // Formula: ReaperPan = (AuditionValue * 2.0) - 1.0
    return (v * 2.0) - 1.0;
}

int convert_sesx_automode_to_reaper(int mode) {
    switch (mode) {
        case 0: return 0; // Off -> Trim/Read
        case 1: return 1; // Read -> Read
        case 2: return 3; // Write -> Write
        case 3: return 4; // Latch -> Latch
        case 4: return 2; // Touch -> Touch
        default: return 0;
    }
}

int kf_type_to_reaper_shape(const std::string& type) {
    if (type == "linear") return 0;
    if (type == "hold") return 1;
    if (type == "bezier") return 5;
    return 0; // Default to linear
}

std::string get_source_format(const std::string& extension, const std::string& fullPath = "") {
    std::string ext = extension;
    for (auto & c: ext) c = tolower(c);

    // Standard Audio Formats
    if (ext == "wav" || ext == "bwf" || ext == "rf64" || ext == "w64" ||
        ext == "aif" || ext == "aiff" || ext == "aifc" || ext == "caf" ||
        ext == "paf" || ext == "pcm" || ext == "raw" || ext == "pvf" ||
        ext == "snd" || ext == "au"  || ext == "sd2" || ext == "avr" ||
        ext == "htk" || ext == "iff" || ext == "mat" || ext == "mpc" ||
        ext == "sds" || ext == "sf"  || ext == "voc" || ext == "vox" ||
        ext == "wve" || ext == "xi") {
        return "WAVE";
    }

    if (ext == "mp3" || ext == "mp2") {
        return "MP3";
    }

    if (ext == "flac") {
        return "FLAC";
    }

    if (ext == "ogg" || ext == "oga") {
        return "VORBIS";
    }

    if (ext == "opus") {
        return "OPUS";
    }

    // Video Containers and Compressed Audio (handled by FFmpeg/VLC/OS Native)
    if (ext == "m4a" || ext == "mp4" || ext == "aac" || ext == "mov" ||
        ext == "m4v" || ext == "3gp" || ext == "3g2" || ext == "avi" ||
        ext == "wmv" || ext == "wma" || ext == "dv"  || ext == "flv" ||
        ext == "mpg" || ext == "mpeg"|| ext == "m1v" || ext == "m2v" ||
        ext == "r3d" || ext == "swf" || ext == "ape" || ext == "ac3" ||
        ext == "ec3") {
        return "VIDEO";
    }
    
    // Fallback using REAPER SDK if the file exists and can be opened
    if (PCM_Source_CreateFromFile && !fullPath.empty()) {
        if (PCM_source* src = PCM_Source_CreateFromFile(fullPath.c_str())) {
            const char* type = src->GetType();
            if (type && *type) {
                std::string res = type;
                delete src;
                return res;
            }
            delete src;
        }
    }

    return "WAVE"; // Final default fallback
}

// --- Importer Logic ---

// Helper to get track audio parameters (mirroring Python's get_track_audio_param)
int get_track_audio_param(pugi::xml_node params, const char* name) {
    std::string val = params.attribute(name).as_string("0");
    if (val == "true") return 1;
    if (val == "false") return 0;
    return atoi(val.c_str());
}

void extract_volume_keyframes(pugi::xml_node parent, std::vector<Keyframe>& keyframes) {
    pugi::xml_node fader = parent.select_node(".//component[@componentID='Audition.Fader'][@name='volume']").node();
    if (!fader) return;
    
    pugi::xml_node volParam = fader.select_node(".//parameter[@name='volume']").node();
    if (!volParam) return;
    
    pugi::xml_node kfContainer = volParam.child("parameterKeyframes");
    if (!kfContainer) return; 
    
    int count = 0;
    for (pugi::xml_node kfNode : kfContainer.children("parameterKeyframe")) {
        Keyframe kf;
        kf.sampleOffset = kfNode.attribute("sampleOffset").as_llong(0);
        kf.value = kfNode.attribute("value").as_double(1.0);
        kf.type = kfNode.attribute("type").as_string("linear");
        keyframes.push_back(kf);
        count++;
    }
}

void extract_pan_keyframes(pugi::xml_node parent, std::vector<Keyframe>& keyframes) {
    pugi::xml_node panner = parent.select_node(".//component[@componentID='Audition.StereoPanner']").node();
    if (!panner) return;
    
    pugi::xml_node panParam = panner.select_node(".//parameter[@name='Pan']").node();
    if (!panParam) return;
    
    pugi::xml_node kfContainer = panParam.child("parameterKeyframes");
    if (!kfContainer) return; 
    
    int count = 0;
    for (pugi::xml_node kfNode : kfContainer.children("parameterKeyframe")) {
        Keyframe kf;
        kf.sampleOffset = kfNode.attribute("sampleOffset").as_llong(0);
        kf.value = kfNode.attribute("value").as_double(0.0);
        kf.type = kfNode.attribute("type").as_string("linear");
        keyframes.push_back(kf);
        count++;
    }
}

void extract_mute_keyframes(pugi::xml_node parent, std::vector<Keyframe>& keyframes) {
    pugi::xml_node mute = parent.select_node(".//component[@componentID='Audition.Mute']").node();
    if (!mute) return;
    
    pugi::xml_node muteParam = mute.select_node(".//parameter[@name='mute']").node();
    if (!muteParam) return;
    
    pugi::xml_node kfContainer = muteParam.child("parameterKeyframes");
    if (!kfContainer) return; 
    
    int count = 0;
    for (pugi::xml_node kfNode : kfContainer.children("parameterKeyframe")) {
        Keyframe kf;
        kf.sampleOffset = kfNode.attribute("sampleOffset").as_llong(0);
        kf.value = kfNode.attribute("value").as_double(0.0);
        kf.type = kfNode.attribute("type").as_string("linear");
        keyframes.push_back(kf);
        count++;
    }
}

// Implementation of WantProjectFile
bool WantProjectFile(const char *fn)
{
    // Check extension
    const char *ext = strrchr(fn, '.');
    if (ext && strcasecmp(ext, ".sesx") == 0) {
        return true;
    }
    return false;
}

// Implementation of EnumFileExtensions
const char *EnumFileExtensions(int i, char **descptr)
{
    if (i == 0) {
        if (descptr) *descptr = (char*)"Adobe Audition Session files (*.sesx)";
        return "SESX";
    }
    return nullptr;
}

// Implementation of LoadProject (Incremental)
int LoadProject(const char *fn, ProjectStateContext *genstate)
{
    if (!genstate) return -1;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(fn);

    if (!result) {
        if (ShowMessageBox) {
            std::string errMsg = "Error: Invalid SESX file.\n";
            errMsg += result.description();
            ShowMessageBox(errMsg.c_str(), "SESX Import Error", 0);
        }
        return -1; // Abort import
    }

    // --- 1. Metadata Extraction ---
    
    // Session Name
    std::string sessionName = "Unknown Session";
    const char *lastSlash = strrchr(fn, '/');
    if (!lastSlash) lastSlash = strrchr(fn, '\\');
    if (lastSlash) sessionName = lastSlash + 1;
    else sessionName = fn;

    // Sample Rate
    int sampleRate = 44100; // Default
    pugi::xml_node sessionNode = doc.select_node("//session").node();
    if (sessionNode) {
        sampleRate = sessionNode.attribute("sampleRate").as_int(44100);
    }

    // Marker/Region Extraction (Parsing embedded XMP)
    std::vector<MarkerInfo> markerList;
    pugi::xml_node xmpNode = doc.select_node("//xmpMetadata").node();
    if (xmpNode) {
        std::string xmpText = xmpNode.child_value();
        size_t pos = xmpText.find("<?xpacket end=\"w\"?>");
        if (pos != std::string::npos) {
            xmpText = xmpText.substr(0, pos);
        }

        pugi::xml_document xmpDoc;
        pugi::xml_parse_result xmpResult = xmpDoc.load_string(xmpText.c_str());
        
        if (xmpResult) {
            pugi::xpath_node_set trackNodes = xmpDoc.select_nodes("//*[local-name()='Tracks']/*[local-name()='Bag']/*[local-name()='li']");
            
            for (pugi::xpath_node track_xpath : trackNodes) {
                pugi::xml_node trackLi = track_xpath.node();
                std::string trackName = trackLi.child("xmpDM:trackName").text().get();
                
                if (trackName == "CuePoint Markers") {
                    pugi::xpath_node_set xmpMarkers = trackLi.select_nodes(".//*[local-name()='markers']/*[local-name()='Seq']/*[local-name()='li']");
                    int mIdx = 1;
                    for (pugi::xpath_node marker_xpath : xmpMarkers) {
                        pugi::xml_node markerLi = marker_xpath.node();
                        MarkerInfo mi;
                        mi.name = markerLi.child("xmpDM:name").text().get();
                        
                        mi.startTime = markerLi.child("xmpDM:startTime").attribute("rdf:value").as_llong(0);
                        if (mi.startTime == 0) mi.startTime = markerLi.child("xmpDM:startTime").text().as_llong(0);
                        
                        mi.duration = markerLi.child("xmpDM:duration").attribute("rdf:value").as_llong(0);
                        if (mi.duration == 0) mi.duration = markerLi.child("xmpDM:duration").text().as_llong(0);
                        
                        mi.index = mIdx++;
                        markerList.push_back(mi);
                    }
                }
            }
        }
    }

    // --- 2. File Resolution ---
    std::map<std::string, FileInfo> fileMap;
    pugi::xpath_node_set fileNodes = doc.select_nodes("//files/file");
    for (pugi::xpath_node xpath_node : fileNodes) {
        pugi::xml_node fileNode = xpath_node.node();
        std::string id = fileNode.attribute("id").as_string();
        FileInfo info;
        info.absolutePath = fileNode.attribute("absolutePath").as_string();
        info.relativePath = fileNode.attribute("relativePath").as_string();
        
        std::string path = info.absolutePath;
        size_t dotPos = path.find_last_of('.');
        if (dotPos != std::string::npos) {
            info.format = get_source_format(path.substr(dotPos + 1), path);
        } else {
            info.format = get_source_format("", path);
        }
        fileMap[id] = info;
    }

    // --- 3. Track & Clip Parsing ---
    std::vector<TrackInfo> tracks;
    pugi::xpath_node_set audioTracks = doc.select_nodes("//audioTrack");

    for (pugi::xpath_node xpath_node : audioTracks) {
        pugi::xml_node audioTrack = xpath_node.node();
        TrackInfo info;

        info.name = audioTrack.child("trackParameters").child("name").text().get();
        if (info.name.empty()) info.name = "Track";

        info.hue = audioTrack.child("trackParameters").attribute("trackHue").as_float(0.0f);
        info.peakcol = hue_to_peakcol(info.hue);

        pugi::xml_node audioParams = audioTrack.child("trackAudioParameters");
        info.solo = get_track_audio_param(audioParams, "solo") != 0;
        info.armed = get_track_audio_param(audioParams, "recordArmed") != 0;
        info.monitoring = get_track_audio_param(audioParams, "monitoring") != 0;
        info.automationMode = get_track_audio_param(audioParams, "automationMode");
        info.mute = false;
        
        pugi::xml_node muteComp = audioTrack.select_node(".//component[@componentID='Audition.Mute']").node();
        if (muteComp) {
            info.mute = muteComp.select_node(".//parameter[@name='mute']").node().attribute("parameterValue").as_int(0) != 0;
        }

        info.volume = 1.0f;
        pugi::xml_node volComp = audioTrack.select_node(".//component[@componentID='Audition.Fader'][@name='volume']").node();
        if (volComp) {
            info.volume = volComp.select_node(".//parameter[@name='volume']").node().attribute("parameterValue").as_float(1.0f);
        }

        info.pan = 0.0f;
        pugi::xml_node panComp = audioTrack.select_node(".//component[@componentID='Audition.StereoPanner']").node();
        if (panComp) {
            info.pan = panComp.select_node(".//parameter[@name='Pan']").node().attribute("parameterValue").as_float(0.0f) / 100.0f;
        }

        extract_volume_keyframes(audioTrack, info.volumeKeyframes);
        extract_pan_keyframes(audioTrack, info.panKeyframes);
        extract_mute_keyframes(audioTrack, info.muteKeyframes);

        pugi::xpath_node_set clipNodes = audioTrack.select_nodes("audioClip");
        for (pugi::xpath_node clip_xpath_node : clipNodes) {
            pugi::xml_node clipNode = clip_xpath_node.node();
            ClipInfo clip;
            clip.name = clipNode.attribute("name").as_string();
            clip.startPoint = clipNode.attribute("startPoint").as_llong(0);
            clip.endPoint = clipNode.attribute("endPoint").as_llong(0);
            clip.sourceInPoint = clipNode.attribute("sourceInPoint").as_llong(0);
            clip.fileID = clipNode.attribute("fileID").as_string();

            // Static clip volume/pan
            clip.volume = 1.0f;
            clip.pan = 0.0f;
            pugi::xml_node cVolComp = clipNode.select_node(".//component[@componentID='Audition.Fader'][@name='volume']").node();
            if (cVolComp) {
                clip.volume = cVolComp.select_node(".//parameter[@name='volume']").node().attribute("parameterValue").as_float(1.0f);
            }
            pugi::xml_node cPanComp = clipNode.select_node(".//component[@componentID='Audition.StereoPanner']").node();
            if (cPanComp) {
                clip.pan = cPanComp.select_node(".//parameter[@name='Pan']").node().attribute("parameterValue").as_float(0.0f) / 100.0f;
            }

            // Fade In
            clip.fadeInDuration = 0.0;
            clip.fadeInShape = 0;
            pugi::xml_node fadeInNode = clipNode.child("fadeIn");
            if (fadeInNode) {
                long long fStart = fadeInNode.attribute("startPoint").as_llong(0);
                long long fEnd = fadeInNode.attribute("endPoint").as_llong(0);
                clip.fadeInDuration = (double)(fEnd - fStart) / sampleRate;
                clip.fadeInShape = fade_type_to_shape(fadeInNode.attribute("type").as_string("lin"));
            }

            // Fade Out
            clip.fadeOutDuration = 0.0;
            clip.fadeOutShape = 0;
            pugi::xml_node fadeOutNode = clipNode.child("fadeOut");
            if (fadeOutNode) {
                long long fStart = fadeOutNode.attribute("startPoint").as_llong(0);
                long long fEnd = fadeOutNode.attribute("endPoint").as_llong(0);
                clip.fadeOutDuration = (double)(fEnd - fStart) / sampleRate;
                clip.fadeOutShape = fade_type_to_shape(fadeOutNode.attribute("type").as_string("lin"));
            }

            extract_volume_keyframes(clipNode, clip.volumeKeyframes);
            extract_pan_keyframes(clipNode, clip.panKeyframes);

            info.clips.push_back(clip);
        }
        tracks.push_back(info);
    }

    // --- 4. Final RPP Generation ---

    genstate->AddLine("<REAPER_PROJECT 0.1");
    
    // Write Markers and Regions
    for (const auto& marker : markerList) {
        double startTime = (double)marker.startTime / sampleRate;
        if (marker.duration == 0) {
            genstate->AddLine("MARKER %d %.6f \"%s\" 0 0 1", marker.index, startTime, marker.name.c_str());
        } else {
            genstate->AddLine("MARKER %d %.6f \"%s\" 1 0 1", marker.index, startTime, marker.name.c_str());
            double endTime = (double)(marker.startTime + marker.duration) / sampleRate;
            genstate->AddLine("MARKER %d %.6f \"\" 1", marker.index, endTime);
        }
    }

    // Write Tracks and Clips
    for (const auto& track : tracks) {
        genstate->AddLine("<TRACK");
        genstate->AddLine("NAME \"%s\"", track.name.c_str());
        genstate->AddLine("PEAKCOL %d", track.peakcol);
        genstate->AddLine("MUTESOLO %d %d 0", track.mute ? 1 : 0, track.solo ? 1 : 0);
        
        char volpanLine[128];
        snprintf(volpanLine, sizeof(volpanLine), "VOLPAN %.6f %.6f -1.000", track.volume, track.pan);
        genstate->AddLine(volpanLine);
        
        genstate->AddLine("REC %d 0 %d 0 0 0 0 0", track.armed ? 1 : 0, track.monitoring ? 1 : 0);
        genstate->AddLine("AUTOMODE %d", convert_sesx_automode_to_reaper(track.automationMode));

        // Track Volume Envelope
        if (!track.volumeKeyframes.empty()) {
            genstate->AddLine("<VOLENV2");
            genstate->AddLine("ACT %d -1", (track.automationMode == 0) ? 0 : 1);
            genstate->AddLine("VIS 1 1 1.0");
            genstate->AddLine("LANEHEIGHT 0 0");
            genstate->AddLine("ARM 0");
            genstate->AddLine("DEFSHAPE 0 -1 -1");
            genstate->AddLine("VOLTYPE 1");
            for (const auto& kf : track.volumeKeyframes) {
                double time = (double)kf.sampleOffset / sampleRate;
                double dummy;
                double val = convert_sesx_volume_to_volenv(kf.value, dummy);
                int shape = kf_type_to_reaper_shape(kf.type);
                genstate->AddLine("PT %.6f %.6f %d 0 0 0", time, val, shape);
            }
            genstate->AddLine(">");
        }

        // Track Pan Envelope
        if (!track.panKeyframes.empty()) {
            genstate->AddLine("<PANENV2");
            genstate->AddLine("ACT %d -1", (track.automationMode == 0) ? 0 : 1);
            genstate->AddLine("VIS 1 1 1.0");
            genstate->AddLine("LANEHEIGHT 0 0");
            genstate->AddLine("ARM 0");
            genstate->AddLine("DEFSHAPE 0 -1 -1");
            for (const auto& kf : track.panKeyframes) {
                double time = (double)kf.sampleOffset / sampleRate;
                double val = convert_sesx_pan_to_reaper(kf.value);
                int shape = kf_type_to_reaper_shape(kf.type);
                genstate->AddLine("PT %.6f %.6f %d 0 0 0", time, val, shape);
            }
            genstate->AddLine(">");
        }

        // Track Mute Envelope
        if (!track.muteKeyframes.empty()) {
            genstate->AddLine("<MUTEENV");
            genstate->AddLine("ACT %d -1", (track.automationMode == 0) ? 0 : 1);
            genstate->AddLine("VIS 1 1 1.0");
            genstate->AddLine("LANEHEIGHT 0 0");
            genstate->AddLine("ARM 0");
            genstate->AddLine("DEFSHAPE 1 -1 -1");

            // Initial State Evaluation
            double firstVal = track.muteKeyframes[0].value;
            int currentState = (firstVal > 0.5) ? 1 : 0;
            double firstTime = (double)track.muteKeyframes[0].sampleOffset / sampleRate;
            genstate->AddLine("PT %.6f %d.000000 1 0 0 0", firstTime, currentState);

            // Pairwise Crossing Calculation
            for (size_t i = 1; i < track.muteKeyframes.size(); ++i) {
                const auto& kf1 = track.muteKeyframes[i - 1];
                const auto& kf2 = track.muteKeyframes[i];

                int newState = (kf2.value > 0.5) ? 1 : 0;

                if (newState != currentState) {
                    // Calculate crossing time: t_cross = t1 + (0.5 - v1) * (t2 - t1) / (v2 - v1)
                    double t1 = (double)kf1.sampleOffset / sampleRate;
                    double t2 = (double)kf2.sampleOffset / sampleRate;
                    double v1 = kf1.value;
                    double v2 = kf2.value;

                    if (std::abs(v2 - v1) > 0.000001) {
                        double t_cross = t1 + (0.5 - v1) * (t2 - t1) / (v2 - v1);
                        genstate->AddLine("PT %.6f %d.000000 1 0 0 0", t_cross, newState);
                    }
                    currentState = newState;
                }
            }
            genstate->AddLine(">");
        }

        for (const auto& clip : track.clips) {
            double position = (double)clip.startPoint / sampleRate;
            double length = (double)(clip.endPoint - clip.startPoint) / sampleRate;
            double soffs = (double)clip.sourceInPoint / sampleRate;

            genstate->AddLine("<ITEM");
            genstate->AddLine("POSITION %.6f", position);
            genstate->AddLine("LENGTH %.6f", length);
            genstate->AddLine("SOFFS %.6f", soffs);
            genstate->AddLine("FADEIN %d %.6f 0.0", clip.fadeInShape, clip.fadeInDuration);
            genstate->AddLine("FADEOUT %d %.6f 0.0", clip.fadeOutShape, clip.fadeOutDuration);
            
            float volpan_v = clip.volume;
            float volpan_p = clip.pan;

            // Clip Volume Envelope
            if (!clip.volumeKeyframes.empty()) {
                volpan_v = 1.0f; // Unity gain when envelope is present
                genstate->AddLine("<VOLENV");
                genstate->AddLine("ACT 1 -1");
                genstate->AddLine("VIS 1 1 1.0");
                genstate->AddLine("LANEHEIGHT 0 0");
                genstate->AddLine("ARM 0");
                genstate->AddLine("DEFSHAPE 0 -1 -1");
                genstate->AddLine("VOLTYPE 1");
                for (const auto& kf : clip.volumeKeyframes) {
                    double time = (double)(kf.sampleOffset - clip.sourceInPoint) / sampleRate;
                    double dummy;
                    double val = convert_sesx_volume_to_volenv(kf.value, dummy);
                    int shape = kf_type_to_reaper_shape(kf.type);
                    genstate->AddLine("PT %.6f %.6f %d 0 0 0", time, val, shape);
                }
                genstate->AddLine(">");
            }

            // Clip Pan Envelope
            if (!clip.panKeyframes.empty()) {
                volpan_p = 0.0f; // Center pan when envelope is present
                genstate->AddLine("<PANENV");
                genstate->AddLine("ACT 1 -1");
                genstate->AddLine("VIS 1 1 1.0");
                genstate->AddLine("LANEHEIGHT 0 0");
                genstate->AddLine("ARM 0");
                genstate->AddLine("DEFSHAPE 0 -1 -1");
                for (const auto& kf : clip.panKeyframes) {
                    double time = (double)(kf.sampleOffset - clip.sourceInPoint) / sampleRate;
                    double val = convert_sesx_pan_to_reaper(kf.value);
                    int shape = kf_type_to_reaper_shape(kf.type);
                    genstate->AddLine("PT %.6f %.6f %d 0 0 0", time, val, shape);
                }
                genstate->AddLine(">");
            }

            genstate->AddLine("VOLPAN %.6f %.6f 1.0 -1.0", volpan_v, volpan_p);
            genstate->AddLine("NAME \"%s\"", clip.name.c_str());
            
            std::string filePath = "";
            std::string fileFormat = "WAVE";
            if (fileMap.count(clip.fileID)) {
                const auto& fileInfo = fileMap.at(clip.fileID);
                filePath = !fileInfo.relativePath.empty() ? fileInfo.relativePath : fileInfo.absolutePath;
                fileFormat = fileInfo.format;
            }

            genstate->AddLine("<SOURCE %s", fileFormat.c_str());
            genstate->AddLine("FILE \"%s\" 1", filePath.c_str());
            genstate->AddLine(">");
            genstate->AddLine(">");
        }
        genstate->AddLine(">");
    }

    genstate->AddLine(">"); // Close <REAPER_PROJECT

    return 0; // Success
}

// The registration structure
static project_import_register_t g_import_reg = {
    WantProjectFile,
    EnumFileExtensions,
    LoadProject
};

extern "C" REAPER_PLUGIN_DLL_EXPORT int REAPER_PLUGIN_ENTRYPOINT(REAPER_PLUGIN_HINSTANCE hInstance, reaper_plugin_info_t *rec)
{
    if (!rec) return 0;
    if (rec->caller_version != REAPER_PLUGIN_VERSION) return 0;

    IMPAPI(ShowConsoleMsg);
    IMPAPI(ShowMessageBox);
    IMPAPI(PCM_Source_CreateFromFile);

    rec->Register("projectimport", &g_import_reg);

    return 1; // Success
}
