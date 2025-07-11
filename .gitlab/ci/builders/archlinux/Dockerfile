#
# Copyright 2013-2025 Software Radio Systems Limited
#
# By using this file, you agree to the terms and conditions set
# forth in the LICENSE file which can be found at the top level of
# the distribution.
#

ARG VERSION=base-20250615.0.365905
FROM archlinux:$VERSION

ADD install_dependencies.sh /usr/local/bin
RUN chmod +x /usr/local/bin/install_dependencies.sh

RUN TZ=Europe/Madrid && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ >/etc/timezone
RUN pacman-key --init \
  && pacman -Sy --noconfirm archlinux-keyring \
  && pacman-key --populate archlinux \
  && pacman -Syu --noconfirm
RUN install_dependencies.sh && pacman -Syu --noconfirm \
  git git-lfs \
  clang \
  ccache valgrind \
  python-pip
RUN git lfs install && pip install --break-system-packages gcovr==5.0

ADD builder.sh /usr/local/bin
RUN chmod +x /usr/local/bin/builder.sh

RUN python -m venv /usr/local/builder_tools
RUN /usr/local/builder_tools/bin/pip install "pandas < 3" "psutil < 6"
ADD changed_tests.py /usr/local/bin
RUN chmod +x /usr/local/bin/changed_tests.py
ADD ram_reporter.py /usr/local/bin
RUN chmod +x /usr/local/bin/ram_reporter.py

WORKDIR /workdir
