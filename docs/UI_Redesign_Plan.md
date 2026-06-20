# CloudSim UI Redesign Plan

## Executive Summary

This document provides a comprehensive UI/UX redesign plan for the CloudSim desktop application. The redesign focuses on modernizing the visual appearance, improving user experience, and establishing a professional design system while maintaining the existing Qt Widgets framework and all functionality.

**Key Principles:**
- Work with existing Qt Widgets stack (no framework migration)
- Preserve all functionality and logic
- Targeted, high-impact improvements
- Maintain Light/Dark theme support

---

## 1. Current State Analysis

### 1.1 Technology Stack
- **Framework:** Qt 5/6 Widgets (C++)
- **Styling:** QPalette with Fusion style
- **Icons:** Custom icon system (UiIcons) with Light/Dark theme variants
- **Layout:** QMainWindow with QDockWidget panels
- **Theme System:** ApplicationStyle namespace with Light/Dark QPalette

### 1.2 Architecture Overview
```
MainWindow (QMainWindow)
├── MenuBar (File, View, Settings)
├── Central Widget
│   └── DocumentTabs (QTabWidget)
│       └── DocumentPage (3D viewport + controls)
├── Left Dock: Property Panel
│   ├── PropertyBrowser (QtTreePropertyBrowser)
│   └── DevicePageWidget
├── Right Dock: Workspace + AI
│   ├── BackendTree (QTreeWidget)
│   ├── RobotSimulationDockWidget
│   ├── SceneTree (QTreeWidget)
│   └── AiAssistantDockWidget
└── Bottom Dock: Runtime Output (RunInfoPage)
```

### 1.3 Identified Design Issues

#### Typography
- **Issue:** System default fonts with no custom typography
- **Impact:** Generic, uninspired appearance
- **Priority:** HIGH

#### Colors
- **Issue:** Basic QPalette colors (plain grays, standard blue highlights)
- **Impact:** Flat, corporate look lacking depth
- **Priority:** HIGH

#### Layout
- **Issue:** Traditional dock widget layout with minimal spacing
- **Impact:** Dense, utilitarian feel
- **Priority:** MEDIUM

#### Interactivity
- **Issue:** Default Qt hover/press states
- **Impact:** Interface feels static
- **Priority:** MEDIUM

#### Components
- **Issue:** Standard Qt widget appearance
- **Impact:** Generic desktop application look
- **Priority:** MEDIUM

---

## 2. Design Audit Findings

### 2.1 Typography Issues

| Issue | Current State | Recommendation |
|-------|---------------|----------------|
| Default fonts | System default (Segoe UI on Windows) | Use `Segoe UI Variable` or add custom font |
| Font weights | Only Regular (400) and Bold (700) | Add Medium (500), SemiBold (600) |
| Letter-spacing | No adjustments | Add tracking for headers, labels |
| Tabular numbers | Not implemented | Enable for data displays (joint angles, coordinates) |

### 2.2 Color and Surface Issues

| Issue | Current State | Recommendation |
|-------|---------------|----------------|
| Background | `rgb(240, 240, 240)` (light), `rgb(53, 53, 53)` (dark) | Use warmer/cooler tints |
| Accent color | `rgb(0, 120, 215)` (blue) | Desaturate slightly, add secondary accent |
| Shadows | No custom shadows | Add tinted, directional shadows |
| Texture | Pure flat surfaces | Add subtle noise/grain overlay |

### 2.3 Layout Issues

| Issue | Current State | Recommendation |
|-------|---------------|----------------|
| Spacing | 8px margins, 8px spacing | Increase to 12-16px for breathing room |
| Container width | No max-width constraint | Add for wide monitors |
| Whitespace | Minimal padding in panels | Increase padding in trees, lists |
| Vertical rhythm | Inconsistent | Establish consistent spacing scale |

### 2.4 Interactivity Issues

| Issue | Current State | Recommendation |
|-------|---------------|----------------|
| Hover states | Default Qt | Add custom hover with background shift |
| Transitions | Instant | Add 200-300ms transitions |
| Focus rings | Default Qt | Enhance visibility |
| Loading states | Basic | Add skeleton loaders |

### 2.5 Component Pattern Issues

| Issue | Current State | Recommendation |
|-------|---------------|----------------|
| Cards/Panels | Default QDockWidget | Custom styled panels with subtle borders |
| Buttons | Default QPushButton | Custom button styles with hover/press |
| Trees | Default QTreeWidget | Custom styling with better spacing |
| Tabs | Default QTabWidget | Modern tab design |

---

## 3. Redesign Recommendations

### 3.1 Typography Improvements

#### 3.1.1 Font Selection
```cpp
// In ApplicationStyle.cpp or new TypographyManager
namespace Typography {
    // Primary font: Segoe UI Variable (Windows 11) or Segoe UI
    const QString PrimaryFont = "Segoe UI Variable";
    const QString FallbackFont = "Segoe UI";
    
    // Monospace for data displays
    const QString MonoFont = "Cascadia Code";
    const QString MonoFallback = "Consolas";
    
    // Font sizes (in pixels)
    constexpr int DisplaySize = 28;
    constexpr int H1Size = 22;
    constexpr int H2Size = 18;
    constexpr int H3Size = 15;
    constexpr int BodySize = 13;
    constexpr int SmallSize = 11;
    constexpr int CaptionSize = 10;
}
```

#### 3.1.2 Font Weight Hierarchy
- **Display/H1:** SemiBold (600)
- **H2/H3:** Medium (500)
- **Body:** Regular (400)
- **Labels/Captions:** Medium (500)

#### 3.1.3 Tabular Numbers for Data
```cpp
// For coordinate displays, joint angles, etc.
QFont monoFont(Typography::MonoFont);
monoFont.setStyleHint(QFont::Monospace);
monoFont.setLetterSpacing(QFont::PercentageSpacing, 100);
```

### 3.2 Color and Surface Improvements

#### 3.2.1 New Color Palette

**Light Theme:**
```cpp
namespace LightTheme {
    // Backgrounds
    const QColor BackgroundPrimary = QColor(248, 248, 250);    // Slight cool tint
    const QColor BackgroundSecondary = QColor(240, 241, 243);  // Panels
    const QColor BackgroundTertiary = QColor(232, 233, 236);   // Cards
    
    // Text
    const QColor TextPrimary = QColor(28, 28, 30);
    const QColor TextSecondary = QColor(99, 99, 102);
    const QColor TextTertiary = QColor(142, 142, 147);
    
    // Accents
    const QColor AccentPrimary = QColor(0, 102, 204);         // Desaturated blue
    const QColor AccentSecondary = QColor(88, 86, 214);       // Purple accent
    
    // Borders
    const QColor BorderLight = QColor(218, 218, 220);
    const QColor BorderMedium = QColor(200, 200, 202);
    
    // Surfaces
    const QColor SurfaceElevated = QColor(255, 255, 255);
    const QColor SurfaceSunken = QColor(238, 239, 242);
}
```

**Dark Theme:**
```cpp
namespace DarkTheme {
    // Backgrounds
    const QColor BackgroundPrimary = QColor(28, 28, 30);
    const QColor BackgroundSecondary = QColor(36, 36, 38);
    const QColor BackgroundTertiary = QColor(44, 44, 46);
    
    // Text
    const QColor TextPrimary = QColor(242, 242, 247);
    const QColor TextSecondary = QColor(142, 142, 147);
    const QColor TextTertiary = QColor(99, 99, 102);
    
    // Accents
    const QColor AccentPrimary = QColor(40, 140, 240);
    const QColor AccentSecondary = QColor(120, 100, 230);
    
    // Borders
    const QColor BorderLight = QColor(56, 56, 58);
    const QColor BorderMedium = QColor(68, 68, 70);
    
    // Surfaces
    const QColor SurfaceElevated = QColor(44, 44, 46);
    const QColor SurfaceSunken = QColor(22, 22, 24);
}
```

#### 3.2.2 Shadow System
```cpp
namespace Shadows {
    // Small shadow for subtle elevation
    const QString Small = "0 1px 2px rgba(0, 0, 0, 0.05)";
    
    // Medium shadow for cards
    const QString Medium = "0 4px 8px rgba(0, 0, 0, 0.08)";
    
    // Large shadow for modals
    const QString Large = "0 12px 24px rgba(0, 0, 0, 0.12)";
    
    // Tinted shadows (match background hue)
    const QString TintedLight = "0 4px 8px rgba(0, 20, 60, 0.06)";
    const QString TintedDark = "0 4px 8px rgba(0, 0, 0, 0.3)";
}
```

#### 3.2.3 Surface Texture
Add subtle noise overlay for depth:
```cpp
// Create a noise texture overlay widget
class NoiseOverlay : public QWidget {
    // Renders subtle grain texture
    // Fixed position, pointer-events-none
};
```

### 3.3 Layout Improvements

#### 3.3.1 Spacing Scale
```cpp
namespace Spacing {
    constexpr int XSmall = 4;
    constexpr int Small = 8;
    constexpr int Medium = 12;
    constexpr int Large = 16;
    constexpr int XLarge = 24;
    constexpr int XXLarge = 32;
}
```

#### 3.3.2 Container Constraints
```cpp
// Add max-width for content areas
constexpr int MaxContentWidth = 1440;
constexpr int MinWindowWidth = 1024;
```

#### 3.3.3 Panel Improvements
- Increase padding in tree widgets: 8px → 12px
- Add consistent 16px padding in dock panels
- Increase spacing between UI sections

#### 3.3.4 Visual Hierarchy
```cpp
// Dock widget styling
QDockWidget {
    border: 1px solid BorderLight;
    border-radius: 8px;
    background: BackgroundSecondary;
}

QDockWidget::title {
    background: BackgroundTertiary;
    padding: 8px 12px;
    font-weight: 500;
}
```

### 3.4 Interactivity Improvements

#### 3.4.1 Hover States
```cpp
// Button hover
QPushButton:hover {
    background: AccentPrimary.lighter(110%);
    transition: background 200ms ease;
}

QPushButton:pressed {
    background: AccentPrimary.darker(110%);
    transform: translateY(1px);
}

// Tree item hover
QTreeWidget::item:hover {
    background: AccentPrimary.lighter(180%);
    border-radius: 4px;
}
```

#### 3.4.2 Transitions
```cpp
// Add animation support
namespace Animation {
    constexpr int DurationFast = 150;
    constexpr int DurationNormal = 200;
    constexpr int DurationSlow = 300;
    
    const QEasingCurve EasingDefault = QEasingCurve::OutCubic;
}
```

#### 3.4.3 Focus States
```cpp
// Enhanced focus rings
*:focus {
    outline: 2px solid AccentPrimary;
    outline-offset: 2px;
}
```

### 3.5 Component Improvements

#### 3.5.1 Modern Button Styles
```cpp
// Primary button
.QPushButton--primary {
    background: AccentPrimary;
    color: white;
    border: none;
    border-radius: 6px;
    padding: 8px 16px;
    font-weight: 500;
}

// Secondary button
.QPushButton--secondary {
    background: transparent;
    color: AccentPrimary;
    border: 1px solid AccentPrimary;
    border-radius: 6px;
    padding: 8px 16px;
}

// Ghost button
.QPushButton--ghost {
    background: transparent;
    color: TextPrimary;
    border: none;
    padding: 8px 12px;
}
```

#### 3.5.2 Modern Tree Widget
```cpp
QTreeWidget {
    background: SurfaceElevated;
    border: 1px solid BorderLight;
    border-radius: 8px;
    padding: 4px;
}

QTreeWidget::item {
    padding: 8px 12px;
    border-radius: 4px;
    margin: 2px 0;
}

QTreeWidget::item:selected {
    background: AccentPrimary;
    color: white;
}

QTreeWidget::item:hover {
    background: BackgroundTertiary;
}
```

#### 3.5.3 Modern Tab Widget
```cpp
QTabWidget::pane {
    border: 1px solid BorderLight;
    border-radius: 8px;
    background: SurfaceElevated;
}

QTabBar::tab {
    background: transparent;
    padding: 8px 16px;
    margin-right: 4px;
    border-bottom: 2px solid transparent;
}

QTabBar::tab:selected {
    background: SurfaceElevated;
    border-bottom: 2px solid AccentPrimary;
    font-weight: 500;
}

QTabBar::tab:hover:!selected {
    background: BackgroundTertiary;
}
```

#### 3.5.4 Property Browser Styling
```cpp
QtTreePropertyBrowser {
    background: SurfaceElevated;
    alternate-background-color: BackgroundSecondary;
}

QtTreePropertyBrowser::item {
    padding: 6px 8px;
    border-bottom: 1px solid BorderLight;
}
```

---

## 4. Implementation Plan

### Phase 1: Foundation (High Impact, Low Risk)
1. **Update color palette** in `ApplicationStyle.cpp`
2. **Add font configuration** with proper weights
3. **Increase spacing** throughout the application
4. **Add custom shadows** for elevation

### Phase 2: Component Styling (Medium Impact)
1. **Create QSS stylesheet** for consistent styling
2. **Style buttons** with hover/press states
3. **Style tree widgets** with modern appearance
4. **Style tab widgets** with active indicators

### Phase 3: Polish (Lower Impact)
1. **Add transitions** for interactive elements
2. **Add noise texture** overlay
3. **Enhance focus states**
4. **Add loading states**

### Phase 4: Advanced (Optional)
1. **Custom title bar**
2. **Floating panels**
3. **Animation system**
4. **Gesture support**

---

## 5. Specific File Changes

### 5.1 `ApplicationStyle.cpp`
- Replace `makeLightPalette()` and `makeDarkPalette()` with new color system
- Add font configuration
- Add shadow definitions

### 5.2 New File: `CloudSimStyles.qss`
- Centralized stylesheet for all widgets
- Consistent styling across the application

### 5.3 `MainWindowUiSetup.cpp`
- Update spacing in layouts
- Apply new widget styles
- Add subtle animations

### 5.4 `UiIcons.h/.cpp`
- Review icon set for consistency
- Consider alternative icon library (Phosphor, Heroicons)

---

## 6. Risk Assessment

| Change | Risk | Mitigation |
|--------|------|------------|
| Color palette update | LOW | Easy to revert, test both themes |
| Font changes | LOW | Fallback to system fonts |
| QSS stylesheet | MEDIUM | Test thoroughly, some widgets may not respond well |
| Layout changes | MEDIUM | Preserve all functionality, test resize behavior |
| Animations | LOW | Optional, can be disabled |

---

## 7. Testing Checklist

- [ ] Light theme displays correctly
- [ ] Dark theme displays correctly
- [ ] All widgets remain functional
- [ ] Property browser editing works
- [ ] Tree selection works
- [ ] Tab switching works
- [ ] Dock undocking/redocking works
- [ ] Window resize maintains layout
- [ ] High DPI scaling works
- [ ] Accessibility (focus states visible)

---

## 8. References

### Design Inspiration
- **VS Code:** Clean, professional dark theme
- **Figma:** Modern UI with subtle shadows
- **Linear:** Minimal, focused interface design

### Qt Styling Resources
- Qt Style Sheets Reference
- Qt Widget Gallery
- QPalette Documentation

---

## 9. Next Steps

1. **Review this plan** with stakeholders
2. **Prioritize changes** based on impact
3. **Create implementation tasks** for each phase
4. **Begin Phase 1** foundation work
5. **Iterate based on feedback**

---

*Document created: 2026-06-18*
*Last updated: 2026-06-18*
