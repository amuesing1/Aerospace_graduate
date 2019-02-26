function plotprob
hold off
models = 0:.1:1;
priors = ones(size(models))/size(models,2)
h = 0; t = 0;
ct = 1;
label = 'priors';
for outcome = [1 0 0 1 0 0 0 1 ]
    subplot(2,4,ct)
    ct
    posterior = generate_posteriors (h, t, priors, models);
    %plot(models, posterior);
    hold off
    plot(models, posterior,'*');
    hold on
    title(label);
    xlabel('model');
    set(gca,'ylim',[0 .3]);
    h = h + outcome;
    t = t + (1-outcome);
    label = 'T';
    if (outcome)
        label = 'H';
    end
    label = sprintf('trial %d: %s',ct,label);
    ct = ct + 1;
end
%print -depsc2 simulate_coin_flip_discrete.eps

function posterior = generate_posteriors(h,t,prior,p)
% compute posterior of hypothesis "p" given "h" heads, "t" tails and prior for p

posterior = prior .* p.^h .* (1-p).^t;
posterior = posterior ./ sum(posterior);