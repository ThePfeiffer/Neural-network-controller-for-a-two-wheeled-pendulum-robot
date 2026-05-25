% bodeplot with linewidth 2
% [h] = bode2(G, wmin, wmax, dbmin, dbmax, amin, amax, dtucolor, showwc, a0offset, hold-on)
%        G is transfer function to plot
%        wmin,wmax     is frequency limits used as parameters to logspace(.)
%        dbmin,dbmax   is dB range for amplitude plot
%        amin,amax     is angle range for fhase plot [deg]
%        dtucolor      is graph color 1=red, blue, dark, yellow,
%                               green, purpel, orange
%        showwc        should wc be calculated and shown? 0=not
%        a0offset      angle subtracted from Phase plot (e.g. 360 if a
%                      hhp zero)
%        hold-on       do not erase old bodeplot
%  returns handle to figure (1234)
%
function [h] = bode2(G, wmin, wmax, dbmin, dbmax, amin, amax, dtuidx, showwc, a0offset, holdon)
h = figure(1234);
w = logspace(wmin,wmax,3000);
[mag,phase,wout] = bode(G,w);
subplot(2,1,1)
hold off
if holdon
  hold on
end
rgb = dtucolor(dtuidx);
semilogx(w, 20*log10(squeeze(mag)), 'color', rgb, 'LineWidth',2)
axis([10^wmin 10^wmax dbmin dbmax])
if dbmax-dbmin > 40
    yt = -100:10:100;
else
    yt = -50:5:50;
end
yticks(yt)
title('Bode plot')
%xlabel('rad/s')
ylabel('Magnitude (dB)')
grid on
if showwc
    hold on
    [Gm,Pm,Wcg,Wcp] = margin(G);
    semilogx([Wcp, Wcp],[dbmin,dbmax],'b');
    hold off
end
%grid MINOR
subplot(2,1,2)
hold off
if holdon
  hold on
end
semilogx(w, squeeze(phase)-a0offset, 'color',rgb, 'LineWidth',2)
axis([10^wmin 10^wmax amin amax])
if amax-amin > 360
    at = round(amin/90)*90-90:90:round(amax/90 + 1)*90;
else
    at = round(amin/30-1)*30:30:round(amax/30 + 1)*30;
end
yticks(at)
ylabel('Phase (deg)')
xlabel('Frequency (rad/s)')
if showwc
    hold on
    %[Gm,Pm,Wcg,Wcp] = margin(G);
    semilogx([Wcp, Wcp],[amin,amax],'b');
    txt = ['\leftarrow \omega_C'];
    %txt = ['\leftarrow \omega_C = ' num2str(Wcp,3)];
    text(Wcp,amin+(amax-amin)*0.75,txt,'fontsize',12)
    hold off
end
grid on
grid minor
end
