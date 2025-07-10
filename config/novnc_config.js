// noVNC Configuration to prevent scrolling
window.novnc_config = {
    // Disable scaling to prevent scrolling
    scaling: 'none',
    
    // Set viewport to match desktop
    viewport: {
        width: 1366,
        height: 768
    },
    
    // Disable mouse scaling
    mouseScaling: false,
    
    // Set quality and performance
    quality: 'medium',
    compression: 'medium',
    
    // Disable clipboard
    clipboard: false,
    
    // Set keyboard layout
    keyboard: 'en-us-qwerty',
    
    // Disable audio
    audio: false,
    
    // Set connection settings
    reconnect: true,
    reconnect_delay: 1000,
    
    // Disable file transfer
    fileTransfer: false,
    
    // Set UI options
    show_dot: false,
    show_quality: false,
    show_bandwidth: false,
    show_clipboard: false,
    show_extra_keys: false,
    show_connect_button: false,
    show_disconnect_button: false,
    show_fullscreen_button: false,
    show_settings_button: false,
    show_keyboard_button: false,
    show_clipboard_button: false,
    show_file_transfer_button: false,
    show_audio_button: false,
    show_connection_info: false,
    show_connection_status: false,
    show_connection_quality: false,
    show_connection_bandwidth: false,
    show_connection_latency: false,
    show_connection_fps: false,
    show_connection_encoding: false,
    show_connection_compression: false,
    show_connection_quality_level: false,
    show_connection_bandwidth_level: false,
    show_connection_latency_level: false,
    show_connection_fps_level: false,
    show_connection_encoding_level: false,
    show_connection_compression_level: false
}; 