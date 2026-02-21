#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
共通設定ファイル
ハードウェア設定とデバイス初期化を一元管理
"""

from luma.core import cmdline
from typing import Any


# ディスプレイ設定
DISPLAY_CONFIG = {
    'interface': 'spi',
    'display': 'ssd1331',
    'width': 96,
    'height': 64,
    'rotate': 2
}

# GPIOピン設定
GPIO_PINS = {
    'button_down': 2,
    'button_up': 3,
    'button_right': 17,
    'button_left': 4
}

# UI設定
UI_CONFIG = {
    'font_size': 10,
    'text_color': 'white',
    'bg_color': 'black',
    'bounce_time': 0.05
}


def create_display_device() -> Any:
    """
    ディスプレイデバイスを作成
    
    Returns:
        luma.core device object
    """
    parser = cmdline.create_parser(description='Display device')
    args = parser.parse_args([
        '--interface', DISPLAY_CONFIG['interface'],
        '--display', DISPLAY_CONFIG['display'],
        '--width', str(DISPLAY_CONFIG['width']),
        '--height', str(DISPLAY_CONFIG['height']),
        '--rotate', str(DISPLAY_CONFIG['rotate'])
    ])
    
    return cmdline.create_device(args)
