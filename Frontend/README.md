# soliris_app

A new Flutter project.

## Getting Started

This project is a starting point for a Flutter application.

A few resources to get you started if this is your first Flutter project:

- [Lab: Write your first Flutter app](https://docs.flutter.dev/get-started/codelab)
- [Cookbook: Useful Flutter samples](https://docs.flutter.dev/cookbook)

For help getting started with Flutter development, view the
[online documentation](https://docs.flutter.dev/), which offers tutorials,
samples, guidance on mobile development, and a full API reference.


# Soliris App

Soliris is a Flutter app that displays real-time and historical wellness data from a wearable, and lets the user manage profile, privacy, display, Wi-Fi, and LED/Vibration settings.
The project focuses on accessibility (text scaling), clear theming (light/dark), and responsive UI.

## Getting Started

This project is a starting point for a Flutter application.

A few resources to get you started if this is your first Flutter project:

- [Lab: Write your first Flutter app](https://docs.flutter.dev/get-started/codelab)
- [Cookbook: Useful Flutter samples](https://docs.flutter.dev/cookbook)

For help getting started with Flutter development, view the
[online documentation](https://docs.flutter.dev/), which offers tutorials,
samples, guidance on mobile development, and a full API reference.


# FRONT-END

## Project map 

assets/
    images/
        unnamed.jpg
lib/
  pages/
    profile_page.dart
    edit_profile_page.dart
    display_settings_page.dart
    privacy_settings_page.dart
    health_dashboard_pages.dart
    wifi_settings_page.dart
    led_vibrations_page.dart
  widgets/
    app_top_bar.dart
    notification_bell.dart
    health_metric_card.dart
    multi_metric_chart.dart
    mood_selector.dart
    day_selector.dart
  theme/
    theme_controller.dart
    scale_utils.dart
  alert_center.dart
  profile_store.dart
  realtime_service.dart


## High-level architecture

- Routing / Navigation: Standard Navigator.push / Navigator.pop between pages.

- State / Data:

    - ThemeController — global theme + text scale (via ValueNotifier), used by all pages.

    - ProfileStore — user profile (name, age, phones, etc.) persisted and exposed as a ValueListenable<UserProfile?>.

    - AlertCenter — central unread alert counter (ValueNotifier<int>) used by the notification bell.

    - RealtimeService — stream of telemetry messages mapped into chart series for the documentation/dashboard page.

- UI building blocks:

    - Fixed, accessible app bars with a contrast-aware back arrow (white on orange, auto for dark mode).

    - Reusable cards, chips, and chart widgets that adapt to text scaling without overflowing.

## Text scaling & accessibility 

- Global scale is controlled by ThemeController.instance.textScale.

- Pages with fixed titles: app bar titles are pinned at textScaler: TextScaler.linear(1.0) so headers don’t explode with large text sizes.

- Clamped areas:

    - Day selector date numbers cap at 150% even if system scale is 160% (prevents clipping).

    - Some compact UI parts (icons/badges) use .sx(context) from scale_utils.dart to scale gently.

## Pages

lib/pages/profile_page.dart — Profile

- Top bar: AppTopBar (brand logo + centered title + notification bell).

- Profile card: shows name, gender, age (via ProfileStore.profile ValueListenable).

- Paired bracelet section with responsive label:

    - If text gets too large or width is tight, “Connected” stacks under “Paired Bracelet”.

- Menu tiles navigate to:

    - LED/Vibrations, Wi-Fi, Display, Privacy.

    - Save success flow: when you return from Edit Profile, the page reads the route result and shows a success banner/snackbar (we return true from EditProfilePage on save).

lib/pages/edit_profile_page.dart — Profile Settings (Edit)

- Header uses the orange app bar with a white back arrow and white title (“Profile Settings”).

- Form controls:

    - Name, phone, gender (dropdown), age.

    - Emergency contact (name/phone).

    - Attending physician (name/phone).

- Save:

    - Builds a new UserProfile copy and calls ProfileStore.instance.save(updated).

    - Navigator.pop(context, true) → lets ProfilePage show “Information successfully saved!”.

lib/pages/privacy_settings_page.dart — Privacy Settings

- Scrollable (fixes previous overflow).

- Orange header with white back arrow and white title (“Privacy Settings”), contrasts correctly in light/dark.

- Content:

    - Lock icon.

    - Title: “Your health, your privacy.”

    - Bullet list explaining anonymization, PII, sharing terms, standards, control/consent.

    - Confirm / Decline buttons. Both pop the page and set consentGiven locally.

lib/pages/display_settings_page.dart — Display

- Header styled like Privacy (orange bar, white back arrow, white title) but the title never scales.

- Theme Mode: radio tiles for Light / Dark → ThemeController.instance.mode.value.

- Text Size: slider (90%–160%).

    - While dragging: page preview text uses live _previewScale.

    - On release: commits globally via ThemeController.instance.setTextScale(v).

- Preview card shows how body/title sizes will look.

lib/pages/wifi_settings_page.dart — Wi-Fi

- Orange app bar with white back arrow + “Wi-Fi Settings”.

- Toggle Wi-Fi, then list available networks.

- Each row: select indicator, SSID, lock (if secured), Wi-Fi icon.

- Selected row highlights (soft orange fill). State is local to the page.

lib/pages/led_vibrations_page.dart — LED / Vibrations

- Orange app bar with white back arrow + “LED / Vibrations”.

- Two switches: enable LEDs, enable vibrations.

- “Signal Codes” list (cards) describing the LED color and vibration pattern for each condition.

lib/pages/health_dashboard_pages.dart — Health Documentation / Dashboard

- Orange app bar (“Health Documentation”), white back arrow (Android shows its own back if not overridden).

- Date selector button (opens date picker).

- Chart card with togglable metrics (see MultiMetricChart).

- “Export Health Summary” → quick PDF stub (logo, date, metric list) using printing + pdf.

## Widgets & Utilities 

lib/widgets/app_top_bar.dart — Brand top bar

- Fixed-size brand logo + centered Soliris title (title size does not scale with textScale to keep layout stable) + NotificationBell.

- Uses .sx(context) from scale_utils.dart to subtly scale paddings/icons.

lib/widgets/notification_bell.dart

- Shows a bell with an unread badge (0..9+).

- Colors invert for dark theme: light badge with orange text on dark, orange badge with white text on light.

- Size gently follows .sx(context).

lib/widgets/health_metric_card.dart

- Compact stat card for “Heart rate”, “SpO₂”, etc.

- Designed to not overflow at large text sizes: layout uses Expanded/Flexible where needed and clamps typography.

lib/widgets/multi_metric_chart.dart

- Full-day multi-line chart using fl_chart.

- Inputs:

    - series: Map<String, List<FlSpot>>

    - visible: Map<String, bool>

    - onToggle(String key, bool selected)

- Features:

    - Canonical metric keys (aliases like heart rate → HR).

    - Color per metric, filled area under lines (optional), now-line (optional).

    - Legend chips (FilterChips) to show/hide series.

    - Legend text is black (not orange) while the dot uses the series color—so it’s legible in any theme.

    - Axis labels at 0h,4h,8h,…,24h; Y from 0..250 (configurable).

lib/widgets/mood_selector.dart

- Four moods in a single row (after testing, we reverted from wrapping to keep symmetry).

- Labels stay readable; icon/touch targets sized with .sx(context) but titles don’t over-scale.

lib/widgets/day_selector.dart

- “Su Mo Tu…” row with circular day numbers.

- Day number clamps at 150% even if global scale is 160% to prevent truncation.

lib/theme/scale_utils.dart

- Adds .sx(context) extension to derive a gentle per-screen scale from MediaQuery.textScaleFactor (useful for paddings, radii, icons), so UI breathes with user settings without breaking.

lib/theme/theme_controller.dart

- Singleton with two ValueNotifiers:

    - mode: ThemeMode — light | dark | system

    - textScale: double — 0.9..1.6

- setTextScale(double) persists and notifies. The MaterialApp listens and rebuilds theme/typography.

lib/profile_store.dart

- UserProfile model + ProfileStore singleton exposing:

    - profile: ValueNotifier<UserProfile?>

    - save(UserProfile) to persist and notify listeners.

- ProfilePage and EditProfilePage both read/write here.

lib/alert_center.dart

- AlertCenter.instance.unread: ValueNotifier<int>

- Bell subscribes and shows the count; pages can call markAllRead() after visiting alerts.

lib/realtime_service.dart

- Simple telemetry stream interface (connect, stream, dispose).

- DocumentationPage / dashboard subscribes, converts incoming messages into chart points.

## Data flow example

Changing theme or text size

1. User toggles Theme Mode or drags Text Size on DisplaySettingsPage.

2. ThemeController updates its notifiers.

3. MaterialApp rebuilds; widgets that read theme/typography update automatically.

4. Small elements relying on .sx(context) scale gently; app bar titles stay fixed.

Editing profile

1. ProfilePage → EditProfilePage (Navigator.push).

2. User edits → Save. ProfileStore.save() persists and Navigator.pop(context, true).

3. Back on ProfilePage, we read the route result and show “Information successfully saved!”.

Privacy consent

1. User visits Privacy Settings and reads bullets.

2. Taps Yes, I confirm or No → sets local consentGiven and pops.

3. You can later read/stash this in an app-level store if needed.

## Theming 

- Primary: Orange (Color(0xFFFF9800)), adjusted for dark variants by Flutter’s ColorScheme.

- Orange app bars with white title/arrow on pages where we want strong brand emphasis.

- Cards use theme.cardColor + scheme.outline for subtle borders, so they look right in both light/dark.

## PDF export (dashboard)

- Uses printing and pdf packages.

- Creates a basic A4 with logo, date, and a placeholder summary. Extend by embedding charts as images later if needed.